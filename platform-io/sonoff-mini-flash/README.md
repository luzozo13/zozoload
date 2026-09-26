# sonoff-mini-flash

ESP8285 (Sonoff Mini) firmware for PZEM-004T v3 power monitoring over MQTT.  
Two independent devices share the same codebase; identity and power topic are injected at compile time.

---

## Hardware

| Item | Detail |
|---|---|
| MCU | Sonoff Mini — ESP8285 (1 MB flash) |
| Power meter | PZEM-004T v3 — HardwareSerial UART0 |
| PZEM wiring | GPIO1 (TX→RX of PZEM), GPIO3 (RX←TX of PZEM) |
| LED | GPIO13 — 1 s blink = MQTT OK, 200 ms = no MQTT |

---

## Devices

| Env | Hostname | OTA IP | Power topic |
|---|---|---|---|
| `sonoff_mini_edf` | `sonoff-mini-pzem-edf` | 192.168.1.15 | `home/power/edf` |
| `sonoff_mini_solar` | `sonoff-mini-pzem-solar` | 192.168.1.41 | `home/power/solar` |

Hostname is **compile-time only** — it cannot be changed at runtime.

---

## Setup

### 1. Credentials

Copy the templates and fill in your values:

```bash
cp include/wifi_credentials_template.h include/wifi_credentials.h
cp include/mqtt_config_template.h      include/mqtt_config.h
```

Edit `include/wifi_credentials.h`:
```cpp
#define WIFI_SSID     "your-ssid"
#define WIFI_PASSWORD "your-password"
```

Edit `include/mqtt_config.h` — set `MQTT_SERVER` to your broker IP.

### 2. Build & upload

```bash
# EDF device
pio run -e sonoff_mini_edf --target upload

# Solar device
pio run -e sonoff_mini_solar --target upload
```

---

## MQTT Topics

### State (retained, published by device)

| Topic | Description |
|---|---|
| `{hostname}/state/pzem` | JSON: `ts, device, V, I, W, kWh, Hz, PF`; or `error: pzem_read_failed` (no answer) / `error: pzem_stale` (frozen reading, see below) |
| `{hostname}/state/comm` | JSON: `ts, device, ip, wifi, mqtt, pzem` |
| `{hostname}/state/time` | JSON: `ts, device, uptime_s` |
| `{hostname}/state/debug` | JSON: `{"debug":"on"}` / `{"debug":"off"}` |
| `{hostname}/state/ota` | OTA progress / confirmation strings, and stale-sensor recovery messages |
| `home/power/edf` or `home/power/solar` | Plain watts float — no hostname prefix |

> `state/pzem`, `state/comm`, `state/time` are only published when **debug is enabled**.  
> `publishPower()` publishes regardless of debug state, but not while the reading is failed or stale.

### Commands (subscribe → device responds)

| Topic | Payload | Effect |
|---|---|---|
| `home/get/debug/all` | (any) | Force-publish all state topics from all devices |
| `{hostname}/set/acquisition-rate` | `"1"` (seconds ≥ 1) | Set PZEM poll interval, saved to EEPROM |
| `{hostname}/set/debug` | `"on"` / `"off"` | Master debug switch — gates state/pzem, state/comm, state/time; saved to EEPROM |
| `{hostname}/set/mqtt/server` | `"192.168.1.22"` | Saved to EEPROM; restart to apply |
| `{hostname}/set/mqtt/port` | `"1883"` | Saved to EEPROM; restart to apply |
| `{hostname}/set/restart` | (any) | Restart device |

---

## Stale-reading watchdog

A live PZEM never returns the exact same `V, I, W, kWh, Hz, PF` for minutes:
voltage alone moves in 0.1 V steps, even at 0 W. If every reading is identical
for **5 min and at least 5 reads**, the reading is treated as frozen:

1. `state/pzem` reports `{"error":"pzem_stale","kWh":…}` and
   `home/power/*` stops publishing (no frozen value is ever republished);
   `state/comm` reports `pzem: fail`.
2. The PZEM driver is re-initialised (`state/ota`: `pzem stale: re-initialising sensor`).
3. If still frozen 5 min later, the ESP restarts
   (`state/ota`: `pzem still stale after re-init: restarting`).

Tune with `-DPZEM_STALE_MS=…` / `-DPZEM_STALE_MIN_READS=…`.

**Why:** PZEM-004T-v30 **1.1.x** (`_lastRead + UPDATE_TIME > millis()` with a
64-bit `_lastRead`) returns its cached values forever once `millis()` wraps
after **49.7 days** of uptime. This matches the ~27 h freeze of both devices
found on 2026-09-26 (identical readings, `kWh` not moving; a restart fixed it). `platformio.ini` now requires **≥ 1.2.1**, which
fixes the comparison upstream (commit `4f8687d`). The watchdog stays as a
safety net for any other stuck read path.

---

## EEPROM

Schema version: **7** — mismatch triggers auto-reset to compile-time defaults.

| Address | Size | Field |
|---|---|---|
| 0 | 64 B | *(reserved — hostname not stored)* |
| 66 | 64 B | `mqttServer` |
| 130 | 2 B | `mqttPort` |
| 133 | 2 B | `pzemAcqRate` |
| 135 | 1 B | `debugEnabled` |
| 500 | 1 B | Schema version |

---

## Compile-time flags (`build_flags`)

| Flag | Effect |
|---|---|
| `-DFORCE_EEPROM_RESET` | Overwrite entire EEPROM with struct defaults on every boot |
| `-DFORCE_DEBUG_ENABLED` | Force `debugEnabled = true`, save to EEPROM |
| `-DFORCE_DEBUG_DISABLED` | Force `debugEnabled = false`, save to EEPROM |
| `-DPZEM_STALE_MS` / `-DPZEM_STALE_MIN_READS` | Stale-reading watchdog thresholds (default 300000 ms / 5 reads) |

Remove the `FORCE_*` flags and reflash after one-time provisioning. Don't
leave `FORCE_DEBUG_DISABLED` in: it hides `state/pzem` on every boot, which is
exactly the topic that shows a frozen sensor.

The debug setting is kept in EEPROM, so a device that was built with
`FORCE_DEBUG_DISABLED` stays silent after reflashing until you send once:

```bash
mosquitto_pub -h 192.168.1.22 -t 'sonoff-mini-pzem-edf/set/debug'   -m on
mosquitto_pub -h 192.168.1.22 -t 'sonoff-mini-pzem-solar/set/debug' -m on
```

---

## Project structure

```
sonoff-mini-flash/
├── src/
│   └── main.cpp
├── include/
│   ├── mqtt_config.h               # active config (not in git)
│   ├── mqtt_config_template.h      # template to copy
│   ├── wifi_credentials.h          # active credentials (not in git)
│   └── wifi_credentials_template.h # template to copy
└── platformio.ini
```

---

## NTP / Timezone

NTP servers: `pool.ntp.org`, `time.nist.gov`.  
Timezone: `CET-1CEST,M3.5.0,M10.5.0/3` (Central European Time with DST).  
Timestamps format: `YYYY-MM-DDTHH:MM:SS` (local time, no Z suffix).  
Falls back to `"unsynced"` until the clock is set (epoch > 24 h).

