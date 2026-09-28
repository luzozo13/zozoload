#!/usr/bin/env python3
"""PWM sweep for zozo-charge: force a fixed charge rate (boost mode, no solar
tracking), step the CP PWM from 110 (max current) up to 220 over one hour once
the car is in state C, then re-check the first level, and record the PZEM
power at each step.

Uses the mosquitto_sub / mosquitto_pub clients (no Python dependencies).
Settings reported on state/debug are saved first and restored (and checked)
at the end, on abort or on Ctrl-C. Needs the firmware with the extended state/debug.
"""
import argparse
import json
import os
import signal
import statistics
import subprocess
import sys
import threading
import time
from datetime import datetime

PZEM_SILENT_ABORT_S = 60
# Keys of state/debug (firmware with the extended payload) that the sweep changes
SETTINGS = ["mode", "charge_speed", "solar_tracking", "charging_enabled", "cheap",
            "debug", "flags", "pzem_pub_rate"]
NOT_C_ABORT_S = 20


def now_str(t=None):
    return datetime.fromtimestamp(t or time.time()).strftime("%Y-%m-%d %H:%M:%S")


class Charger:
    def __init__(self, host, port, base, raw_log):
        self.host, self.port, self.base = host, port, base
        self.raw_log = raw_log
        self.lock = threading.Lock()
        self.state = None            # last EVSE state letter from state/details or state/change
        self.last_c_ts = 0.0         # last time state C was reported
        self.solar_tracking = None
        self.debug = None            # last state/debug payload
        self.pzem = []               # (recv_time, payload dict)
        self.acks = []               # (recv_time, value) from set/charge_rate/status
        self.stop = threading.Event()
        self.proc = subprocess.Popen(
            ["mosquitto_sub", "-h", host, "-p", str(port), "-v", "-F", "%U %t %p",
             "-t", f"{base}/#"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, bufsize=1)
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self):
        for line in self.proc.stdout:
            parts = line.rstrip("\n").split(" ", 2)
            if len(parts) < 3:
                continue
            ts, topic, payload = float(parts[0]), parts[1], parts[2]
            self.raw_log.write(f"[{now_str(ts)}] {topic} {payload}\n")
            self.raw_log.flush()
            try:
                data = json.loads(payload)
            except ValueError:
                data = None
            with self.lock:
                if topic == f"{self.base}/state/details" and isinstance(data, dict):
                    self.state = data.get("state")
                    if self.state == "C":
                        self.last_c_ts = ts
                    self.solar_tracking = data.get("solar_tracking")
                elif topic == f"{self.base}/state/change" and isinstance(data, dict) and "to" in data:
                    self.state = data["to"]
                    if self.state == "C":
                        self.last_c_ts = ts
                elif topic == f"{self.base}/state/pzem" and isinstance(data, dict):
                    self.pzem.append((ts, data))
                elif topic == f"{self.base}/set/charge_rate/status" and isinstance(data, dict):
                    self.acks.append((ts, data.get("value")))
                elif topic == f"{self.base}/state/debug" and isinstance(data, dict):
                    self.debug = data
        self.stop.set()

    def pub(self, suffix, payload):
        topic = f"{self.base}{suffix}"
        subprocess.run(["mosquitto_pub", "-h", self.host, "-p", str(self.port), "-t", topic, "-m", str(payload)],
                       check=True)
        self.raw_log.write(f"[{now_str()}] >>> {topic} {payload}\n")
        self.raw_log.flush()

    def close(self):
        self.proc.terminate()


def wait_for(pred, timeout, poll=1.0):
    end = time.time() + timeout
    while time.time() < end:
        if pred():
            return True
        time.sleep(poll)
    return False


def read_settings(ch, timeout=10):
    """Fresh state/debug from the charger (per-device get, not the home/ broadcast)."""
    with ch.lock:
        ch.debug = None
    ch.pub("/get/debug", "1")
    if not wait_for(lambda: ch.debug is not None, timeout, 0.2):
        return None
    with ch.lock:
        return dict(ch.debug)


def restore_settings(ch, snap, outdir):
    """Put back what state/debug reported before the sweep, then read it again to check."""
    print("Restoring settings...")
    ch.pub("/set/mode", snap["mode"])  # also resets solar_tracking / charging_enabled / speed
    implied_tracking = "on" if snap["mode"] == "solar" else "off"
    if snap["solar_tracking"] != implied_tracking:
        ch.pub("/set/solar_tracking", snap["solar_tracking"])
    ch.pub("/set/cheap", "on" if snap["cheap"] else "off")
    if snap["mode"] != "cheap" and not snap["charging_enabled"]:
        ch.pub("/set/delay", "on")
    if snap["mode"] == "boost":
        ch.pub("/set/charge_rate", snap["charge_speed"])
    ch.pub("/set/debug/pzem/rate", snap["pzem_pub_rate"])
    ch.pub("/set/debug_flags", snap["flags"])
    ch.pub("/set/debug", snap["debug"])
    time.sleep(1)
    after = read_settings(ch)
    if after is None:
        print("WARNING: no state/debug after restore, check the charger manually")
        return
    with open(os.path.join(outdir, "settings_after.json"), "w") as f:
        json.dump(after, f, indent=1)
    # In solar/cheap mode the firmware's own loop owns the charge speed
    keys = [k for k in SETTINGS if k != "charge_speed" or snap["mode"] == "boost"]
    diff = {k: (snap.get(k), after.get(k)) for k in keys if snap.get(k) != after.get(k)}
    if diff:
        print("WARNING: not restored (before, after): " + json.dumps(diff))
    else:
        print("Settings restored and verified: " + ", ".join(f"{k}={after[k]}" for k in keys))


def pct(sorted_vals, f):
    return sorted_vals[int(f * (len(sorted_vals) - 1))]


def summarize(levels, pzem, settle_s):
    rows = []
    for i, lv in enumerate(levels):
        t0 = lv["ack"] or lv["sent"]
        t1 = levels[i + 1]["sent"] if i + 1 < len(levels) else lv["end"]
        samples = [d for ts, d in pzem if t0 + settle_s <= ts < t1 and d.get("W") is not None]
        w = sorted(d["W"] for d in samples)
        row = {"pwm": lv["pwm"], "start": now_str(lv["sent"]), "acked": lv["ack"] is not None, "n": len(w),
               "recheck": lv.get("recheck", False)}
        if w:
            row.update(W_median=round(statistics.median(w), 1), W_p10=pct(w, .1), W_p90=pct(w, .9),
                       W_min=w[0], W_max=w[-1],
                       I_median=round(statistics.median(d["I"] for d in samples), 2),
                       V_median=round(statistics.median(d["V"] for d in samples), 1),
                       PF_median=round(statistics.median(d["PF"] for d in samples), 2))
        rows.append(row)
    # Offered current rises as PWM falls, so the draw should too. Walk the levels
    # from the highest PWM down and flag one that draws less than a higher-PWM level.
    sweep = [r for r in rows if not r.get("recheck")]
    best = None
    for row in sorted(sweep, key=lambda r: -r["pwm"]):
        m = row.get("W_median")
        row["flag"] = "no settled data" if m is None else ""
        if m is not None and best is not None and m < best:
            row["flag"] = "draw below a higher-PWM level: car limiting?"
        if m is not None:
            best = m if best is None else max(best, m)
    # Re-check: back at the first PWM at the end. Outside the first run's p10-p90
    # means the car's draw changed during the sweep (e.g. taper near full).
    for row in rows:
        if not row.get("recheck"):
            continue
        ref = sweep[0] if sweep else {}
        m = row.get("W_median")
        if m is None or ref.get("W_median") is None:
            row["flag"] = "re-check: no settled data"
        elif ref["W_p10"] <= m <= ref["W_p90"]:
            row["flag"] = f"re-check of first level: {m / ref['W_median']:.1%} of start, within start p10-p90"
        else:
            row["flag"] = (f"re-check of first level: {m / ref['W_median']:.1%} of start, OUTSIDE start "
                           f"p10-p90: car draw changed during sweep")
    return rows


def write_summary(outdir, rows):
    cols = ["pwm", "recheck", "start", "acked", "n", "W_median", "W_p10", "W_p90", "W_min", "W_max",
            "I_median", "V_median", "PF_median", "flag"]
    with open(os.path.join(outdir, "summary.csv"), "w") as f:
        f.write(",".join(cols) + "\n")
        for r in rows:
            f.write(",".join(str(r.get(c, "")) for c in cols) + "\n")
    md = ["| pwm | n | median W | p10–p90 W | min–max W | I (A) | V | PF | flag |",
          "|---:|---:|---:|---:|---:|---:|---:|---:|---|"]
    for r in rows:
        label = f"{r['pwm']}{' (re-check)' if r.get('recheck') else ''}"
        if r.get("W_median") is None:
            md.append(f"| {label} | {r['n']} | — | — | — | — | — | — | {r['flag']} |")
        else:
            md.append(f"| {label} | {r['n']} | {r['W_median']:.0f} | {r['W_p10']:.0f}–{r['W_p90']:.0f} | "
                      f"{r['W_min']:.0f}–{r['W_max']:.0f} | {r['I_median']} | {r['V_median']} | "
                      f"{r['PF_median']} | {r['flag']} |")
    text = "\n".join(md) + "\n"
    with open(os.path.join(outdir, "summary.md"), "w") as f:
        f.write(text)
    return text


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="192.168.1.22")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--base", default="zozo-charge", help="topic base (hostname of the charger)")
    ap.add_argument("--start", type=int, default=110, help="first PWM (110 = most current)")
    ap.add_argument("--end", type=int, default=220, help="last PWM (220 = least current); either direction works")
    ap.add_argument("--no-recheck", dest="recheck", action="store_false",
                    help="skip the final step back at the first PWM (taper check)")
    ap.add_argument("--step", type=int, default=5)
    ap.add_argument("--duration", type=float, default=3600, help="total sweep time, seconds")
    ap.add_argument("--settle", type=float, default=30, help="seconds ignored after each PWM change")
    ap.add_argument("--pzem-rate", type=int, default=5, help="state/pzem publish interval during the sweep (>=5)")
    ap.add_argument("--wait-timeout", type=float, default=12 * 3600, help="max wait for state C, seconds")
    ap.add_argument("--out", default=".", help="directory in which the run folder is created")
    ap.add_argument("--dry-run", action="store_true", help="print the plan and exit")
    a = ap.parse_args()

    if a.start == a.end or a.step <= 0 or a.pzem_rate < 5:
        ap.error("need start != end, step > 0, pzem-rate >= 5")
    if not all(0 < p < 255 for p in (a.start, a.end)):
        ap.error("start/end must be within 1..254")
    d = a.step if a.end > a.start else -a.step
    pwms = list(range(a.start, a.end + (1 if d > 0 else -1), d))
    if pwms[-1] != a.end:
        pwms.append(a.end)
    n_levels = len(pwms) + (1 if a.recheck else 0)
    dwell = a.duration / n_levels
    print(f"Plan: {len(pwms)} levels {pwms[0]}→{pwms[-1]} step {a.step}"
          f"{f' + re-check at {pwms[0]}' if a.recheck else ''}, {dwell:.0f} s each "
          f"({a.duration / 60:.0f} min), first {a.settle:.0f} s of each level ignored, "
          f"PZEM every {a.pzem_rate} s → ~{max(0, int((dwell - a.settle) / a.pzem_rate))} samples/level.")
    if dwell <= a.settle + 2 * a.pzem_rate:
        ap.error("dwell too short for the settle time and PZEM rate")
    if a.dry_run:
        return

    outdir = os.path.join(a.out, "pwm_sweep_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    os.makedirs(outdir)
    raw = open(os.path.join(outdir, "raw.log"), "w")
    ch = Charger(a.host, a.port, a.base, raw)
    snap = None
    levels = []
    abort_reason = None

    def on_signal(signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, on_signal)
    try:
        time.sleep(1)
        if ch.stop.is_set():
            sys.exit(f"mosquitto_sub exited: {ch.proc.stderr.read().strip()}")
        snap = read_settings(ch)
        if snap is None:
            abort_reason = "no state/debug reply (charger offline?)"
            pwms = []
        elif any(k not in snap for k in SETTINGS):
            snap = None
            abort_reason = "state/debug lacks mode/pzem_pub_rate: charger firmware too old to snapshot"
            pwms = []
        else:
            with open(os.path.join(outdir, "settings_before.json"), "w") as f:
                json.dump(snap, f, indent=1)
            print("Settings saved: " + ", ".join(f"{k}={snap[k]}" for k in SETTINGS))
            ch.pub("/set/debug/pzem/state", "on")
            ch.pub("/set/debug/pzem/rate", a.pzem_rate)

            print(f"{now_str()} Waiting for the car to report charging (state C)...")
            if not wait_for(lambda: ch.state == "C", a.wait_timeout):
                abort_reason = "car never reached state C"
                pwms = []
            else:
                print(f"{now_str()} State C. Forcing boost + PWM {pwms[0]}.")
                ch.pub("/set/mode", "boost")
        steps = [(p, False) for p in pwms] + ([(pwms[0], True)] if a.recheck and pwms else [])
        for pwm, is_recheck in steps:
            sent = time.time()
            n_acks = len(ch.acks)
            ch.pub("/set/charge_rate", pwm)
            lv = {"pwm": pwm, "sent": sent, "ack": None, "end": None, "recheck": is_recheck}
            levels.append(lv)
            if wait_for(lambda: len(ch.acks) > n_acks, 10, 0.2):
                ack_t, ack_v = ch.acks[-1]
                lv["ack"] = ack_t if ack_v == pwm else None
            print(f"{now_str()} PWM {pwm}{' (re-check)' if is_recheck else ''} "
                  f"{'acked' if lv['ack'] else 'NOT acked'}", flush=True)
            level_end = sent + dwell
            while time.time() < level_end:
                time.sleep(1)
                t = time.time()
                with ch.lock:
                    last_pzem = ch.pzem[-1][0] if ch.pzem else 0
                    last_w = ch.pzem[-1][1].get("W") if ch.pzem else None
                    state, last_c = ch.state, ch.last_c_ts
                if state != "C" and t - last_c >= NOT_C_ABORT_S:
                    abort_reason = f"car left state C (state {state})"
                elif t - max(last_pzem, sent) > PZEM_SILENT_ABORT_S:
                    abort_reason = f"no state/pzem for {PZEM_SILENT_ABORT_S} s"
                if abort_reason:
                    break
                if int(t - sent) % 30 == 0:
                    print(f"  {now_str()} pwm={pwm} W={last_w} state={state}", flush=True)
            lv["end"] = time.time()
            if abort_reason:
                break
    except KeyboardInterrupt:
        abort_reason = "interrupted"
        if levels and levels[-1]["end"] is None:
            levels[-1]["end"] = time.time()
    finally:
        if snap is not None:
            restore_settings(ch, snap, outdir)
        ch.close()
        raw.close()

    if abort_reason:
        print(f"ABORTED: {abort_reason}")
    if levels:
        with ch.lock:
            pzem = list(ch.pzem)
        with open(os.path.join(outdir, "samples.csv"), "w") as f:
            f.write("recv_time,pwm_commanded,V,I,W,PF\n")
            for ts, d in pzem:
                cur = [lv["pwm"] for lv in levels if lv["sent"] <= ts]
                f.write(f"{now_str(ts)},{cur[-1] if cur else ''},{d.get('V')},{d.get('I')},{d.get('W')},{d.get('PF')}\n")
        print(write_summary(outdir, summarize(levels, pzem, a.settle)))
    print(f"Files in {outdir}")


if __name__ == "__main__":
    main()
