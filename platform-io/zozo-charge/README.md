# Zozo Charge

## Configuration Guide

Before compiling and running this project, you need to configure your WiFi and MQTT settings.  
Sensitive files are not tracked by git for security reasons. Follow these steps:

---

### 1. WiFi Credentials

1. Copy the template file:
    ```sh
    cp include/wifi_credentials_template.h include/wifi_credentials.h
    ```
2. Edit `include/wifi_credentials.h` and replace:
    - `<WIFI_SSID>` with your WiFi network name
    - `<WIFI_PASSWORD>` with your WiFi password

---

### 2. MQTT Configuration

1. Copy the template file:
    ```sh
    cp include/mqtt_config_template.h include/mqtt_config.h
    ```
2. Edit `include/mqtt_config.h` and replace:
    - `MQTT_SERVER` with your MQTT broker address (e.g. `"192.168.1.22"`)
    - `MQTT_PORT` with your MQTT broker port (default: `1883`)
    - `MQTT_TOPIC` with your desired MQTT topic base (e.g. `"zozo-charge"`)

---

**Note:**  
- Do **not** commit your personal `wifi_credentials.h` or `mqtt_config.h` files.  
- These files are listed in `.gitignore` for your privacy.

If you have any questions, please refer to the template files or open an issue.