# Zozo Charge - ESP32 EVSE Controller with PZEM-004T Telemetry

A class-based Arduino/PlatformIO project for controlling an EV charger (EVSE) with integrated PZEM-004T power monitoring and MQTT remote control/telemetry.

## Features

- **EVSE Control**: Full EV charger state management (A, B, C, fault states)
- **PZEM-004T Telemetry**: Real-time electrical measurements:
  - Voltage (V)
  - Current (A)
  - Power (W)
  - Energy (kWh)
  - Frequency (Hz)
  - Power Factor
- **MQTT Integration**: 
  - Remote control via MQTT
  - Real-time telemetry publishing
  - Debug messages for troubleshooting
- **OTA Updates**: Over-the-Air firmware updates via ArduinoOTA
- **Clean Architecture**: Modular class-based design (EVSEController, MqttHandler)

---

## Hardware Setup

### Board
- **ESP32** (tested on TTGO T7-Mini v1.4+)
- **PZEM-004T v3.0** power meter module

### Connections

#### PZEM Module
- **ESP32 Pin 1 (TXD)** → PZEM RX
- **ESP32 Pin 3 (RXD)** → PZEM TX
- **ESP32 GND** → PZEM GND
- **PZEM 5V** → Dedicated 5V power supply (not ESP32 VCC)
- **PZEM AC terminals** → 230V AC mains

**Important**: Power the PZEM from a separate 5V supply, not from the ESP32, to avoid brownout issues.

#### Control Pins
Refer to `include/params.h` for all pin definitions (relays, LEDs, buttons, etc.)

---

## Configuration Guide

Before compiling, configure WiFi and MQTT settings:

### 1. WiFi Credentials

1. Copy the template:
    ```sh
    cp include/wifi_credentials_template.h include/wifi_credentials.h
    ```
2. Edit `include/wifi_credentials.h`:
    ```cpp
    #define WIFI_SSID "YourNetworkName"
    #define WIFI_PASSWORD "YourPassword"
    ```

### 2. MQTT Configuration

1. Copy the template:
    ```sh
    cp include/mqtt_config_template.h include/mqtt_config.h
    ```
2. Edit `include/mqtt_config.h`:
    ```cpp
    #define MQTT_SERVER "192.168.1.100"  // Replace with your MQTT broker IP
    #define MQTT_PORT   1883
    #define MQTT_TOPIC  "evse-topic"       // Replace with your desired topic
    ```

### 3. MQTT Topics

All topics below are prefixed with `MQTT_TOPIC` (see `include/mqtt_config.h`).

**State (published by the charger):**
- `state/details` - Periodic EVSE state: `state` (A, B, C, S = sleeping, F = fault), `CPP_max`/`CPP_min`, `charging_enabled`, `solar_tracking`, `solar_paused`
- `state/change` - State transitions (`{"from":"B","to":"C"}`) and events (`{"event":"cp_fault_seen",...}`)
- `state/pwm` - Current PWM and setpoint (retained)
- `state/solar_tracking` - Solar control loop telemetry: `action` (`hold`, `down`, `up`, `stale`, `pause`, `paused`, `resume`), `deficit_s`, `paused_s`, `resume_s`
- `state/pzem` - PZEM telemetry (V, I, W, kWh, Hz, PF)
- `state/comm`, `state/time`, `state/debug` - Connection, time and debug status

**Commands (subscribed by the charger):**
- `set/mode` - `boost` | `solar` | `cheap`
- `set/cheap` - `on` | `off` (cheap-tariff window, used by `cheap` mode)
- `set/charge_rate` - Charging PWM (0-255, lower = more current)
- `set/delay` - `on` (hold charging) | `off`
- `set/solar_tracking` - `on` | `off`
- `set/debug`, `set/debug_flags`, `get/debug` - Debug output control

Solar production and house consumption are read from the topics set in
`MQTT_SOLAR_PRODUCTION_POWER` / `MQTT_HOUSE_CONSUMPTION_POWER`.

### 4. Charging Modes

- **boost** - Charge at 16A as soon as the car is ready.
- **cheap** - Charge at 16A only while `set/cheap` is `on`.
- **solar** - Closed-loop tracking of solar production (see `SOLAR_*` in `include/params.h`):
  - the PWM ramps up slowly on surplus and backs off fast on deficit, between 8A and `SOLAR_PWM_MIN`;
  - when the charger is already at minimum current and still in deficit (or solar data is stale) for `SOLAR_PAUSE_AFTER_MS`, charging is **paused** (J1772 stop sequence), but never less than `SOLAR_MIN_ON_MS` after charging started;
  - a pause lasts at least `SOLAR_MIN_PAUSE_MS`, and charging **resumes** only after production has stayed above `SOLAR_RESUME_W` for `SOLAR_RESUME_AFTER_MS`. It restarts at minimum current and ramps up;
  - unplugging the car or selecting a mode clears the pause.

### 5. Safety Behaviour

- The EVSE state machine keeps running without WiFi or MQTT: reconnection is non-blocking.
- A task watchdog (`WDT_TIMEOUT_S`) reboots the board if the main loop hangs. At boot the relay is open and the CP is at +12V.
- CP below the state C threshold (state D/E, e.g. CP shorted): with `FAULT_DETECT_ENFORCE 0` (default) it is only reported as a `cp_fault_seen` event on `state/change`. With `1` the charger goes to fault (relay open, `FLT_CTRL` high) and may clear after `FAULT_RETRY_MS` once the CP reads normal again.

---

## Compilation & Upload

### Build
```bash
pio run
```

### Upload via USB
```bash
pio run --target upload
```

### OTA Upload (after first USB upload)
```bash
pio run --target upload --upload-port 192.168.1.50
```
(Replace `192.168.1.50` with your ESP32's actual IP address)

---

## Project Structure

```
include/
  params.h                 - Pin definitions & settings
  EVSEController.h         - EVSE state machine
  MqttHandler.h            - MQTT operations
  mqtt_config_template.h   - MQTT template (copy to mqtt_config.h)
  wifi_credentials_template.h - WiFi template (copy to wifi_credentials.h)

src/
  main.cpp                 - Entry point, PZEM management
  EVSEController.cpp       - EVSE logic
  MqttHandler.cpp          - MQTT publish/subscribe
```

---

## Dependencies

- PubSubClient (MQTT)
- mandulaj/PZEM-004T-v30 (PZEM meter)
- ArduinoOTA (Over-The-Air updates)

---

## Security Notes

- ⚠️ **Do not commit** `wifi_credentials.h` or `mqtt_config.h` to git
- These files are in `.gitignore` by default
- Always use the template files as references
- Update `.gitignore` if you add new sensitive config files

---

## Troubleshooting

### PZEM Not Communicating
1. Verify wiring: ESP32 pins 1→PZEM TX, 3→PZEM RX
2. Check PZEM is powered from separate 5V supply
3. Monitor debug topics: `zozo-charge/debug/comm`
4. Verify PZEM address (default: 0x04)

### ESP32 Won't Boot When PZEM Connected
- Ensure 10kΩ pull-up resistor on GPIO2 if used
- Power PZEM from external 5V, not ESP32 VCC
- Check for shorts in wiring

### MQTT Not Connecting
1. Verify `mqtt_config.h` has correct broker IP/port
2. Check WiFi connection first
3. Monitor debug topics for status

---

## License

Your license here

## Contributors

Developed by the ZoZoload team
