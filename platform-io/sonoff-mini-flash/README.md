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
| `sonoff_mini_edf` | `sonoff-mini-pzem-edf` | 192.168.1.15 | `/home/power/edf` |
| `sonoff_mini_solarprod` | `sonoff-mini-pzem-solar` | 192.168.1.41 | `/home/power/solar` |

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
pio run -e sonoff_mini_solarprod --target upload
```

---

## MQTT Topics

### State (retained, published by device)

| Topic | Description |
|---|---|
| `{hostname}/state/pzem` | JSON: `ts, device, V, I, W, kWh, Hz, PF` (or `error` on read fail) |
| `{hostname}/state/comm` | JSON: `ts, device, ip, wifi, mqtt, pzem` |
| `{hostname}/state/time` | JSON: `ts, device, uptime_s` |
| `{hostname}/state/debug` | JSON: `{"debug":"on"}` / `{"debug":"off"}` |
| `{hostname}/state/ota` | OTA progress / confirmation strings |
| `/home/power/edf` or `/home/power/solar` | Plain watts float — no hostname prefix |

> `state/pzem`, `state/comm`, `state/time` are only published when **debug is enabled**.  
> `publishPower()` always publishes regardless of debug state.

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

Remove the flag and reflash after one-time provisioning.

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

