//-- MQTT Handler Implementation --//

#include "MqttHandler.h"
#include "EVSEController.h"
#include "params.h"
#include "mqtt_config.h"
#include "wifi_credentials.h"
#include <Arduino.h>
#include <WiFi.h>
#include <stdio.h>
#include <time.h>

MqttHandler::MqttHandler(PubSubClient& client)
    : m_pubsubClient(&client), m_evseController(nullptr)
{
    strncpy(m_hostname,    NVS_DEFAULT_HOSTNAME,  sizeof(m_hostname) - 1);
    strncpy(m_mqtt_server, MQTT_SERVER,            sizeof(m_mqtt_server) - 1);
    m_pzem_addr     = NVS_DEFAULT_PZEM_ADDR;
    m_pzem_acq_rate = NVS_DEFAULT_PZEM_ACQ_RATE;
    m_pzem_pub_rate = NVS_DEFAULT_PZEM_PUB_RATE;
    m_mqtt_port     = MQTT_PORT;
    m_last_pzem_json[0] = '\0';
}

void MqttHandler::setup(const char* server, int port) {
    if (m_pubsubClient) {
        m_pubsubClient->setServer(server, port);
    }
}

void MqttHandler::setup() {
    setup(m_mqtt_server, m_mqtt_port);
}

String MqttHandler::ht(const char* suffix) {
    return String(m_hostname) + suffix;
}

void MqttHandler::setEVSEController(EVSEController& controller) {
    m_evseController = &controller;
}

void MqttHandler::loadFromNVS() {
    Preferences prefs;
    prefs.begin("zozo", false);
    uint8_t schema = prefs.getUChar("schema", 0);
    if (schema != NVS_SCHEMA_VERSION) {
        // New firmware version or first boot: reset all to defaults
        m_debug_enabled = true;
        m_debug_flags   = DBG_DEFAULT_FLAGS;
        strncpy(m_hostname,    NVS_DEFAULT_HOSTNAME,  sizeof(m_hostname) - 1);
        strncpy(m_mqtt_server, MQTT_SERVER,            sizeof(m_mqtt_server) - 1);
        m_pzem_addr     = NVS_DEFAULT_PZEM_ADDR;
        m_pzem_acq_rate = NVS_DEFAULT_PZEM_ACQ_RATE;
        m_pzem_pub_rate = NVS_DEFAULT_PZEM_PUB_RATE;
        m_mqtt_port     = MQTT_PORT;
        prefs.putBool("dbg_en",     m_debug_enabled);
        prefs.putUChar("dbg_flags", m_debug_flags);
        prefs.putString("hostname",  m_hostname);
        prefs.putString("mqtt_srv",  m_mqtt_server);
        prefs.putUChar("pzem_addr",  m_pzem_addr);
        prefs.putUShort("pzem_acq",  m_pzem_acq_rate);
        prefs.putUShort("pzem_pub",  m_pzem_pub_rate);
        prefs.putUShort("mqtt_port", m_mqtt_port);
        prefs.putUChar("schema",     NVS_SCHEMA_VERSION);
    } else {
        m_debug_flags   = prefs.getUChar("dbg_flags", DBG_DEFAULT_FLAGS);
        m_debug_enabled = prefs.getBool("dbg_en",     true);
        String h = prefs.getString("hostname", NVS_DEFAULT_HOSTNAME);
        strncpy(m_hostname, h.c_str(), sizeof(m_hostname) - 1);
        String s = prefs.getString("mqtt_srv", MQTT_SERVER);
        strncpy(m_mqtt_server, s.c_str(), sizeof(m_mqtt_server) - 1);
        m_pzem_addr     = prefs.getUChar("pzem_addr",  NVS_DEFAULT_PZEM_ADDR);
        m_pzem_acq_rate = prefs.getUShort("pzem_acq",  NVS_DEFAULT_PZEM_ACQ_RATE);
        m_pzem_pub_rate = prefs.getUShort("pzem_pub",  NVS_DEFAULT_PZEM_PUB_RATE);
        m_mqtt_port     = prefs.getUShort("mqtt_port", MQTT_PORT);
    }
    prefs.end();
}

void MqttHandler::saveConfig() {
    Preferences prefs;
    prefs.begin("zozo", false);
    prefs.putString("hostname",  m_hostname);
    prefs.putString("mqtt_srv",  m_mqtt_server);
    prefs.putUChar("pzem_addr",  m_pzem_addr);
    prefs.putUShort("pzem_acq",  m_pzem_acq_rate);
    prefs.putUShort("pzem_pub",  m_pzem_pub_rate);
    prefs.putUShort("mqtt_port", m_mqtt_port);
    prefs.end();
}

bool MqttHandler::reconnect() {
    if (!m_pubsubClient) return false;
    if (m_pubsubClient->connect(m_hostname)) {
        // Existing EV-charger topics
        subscribe(MQTT_SET_CHARGE_RATE);
        subscribe(MQTT_SET_DELAY);
        subscribe(MQTT_SET_DEBUG);
        subscribe(MQTT_SET_DEBUG_FLAGS);
        subscribe(MQTT_GET_DEBUG);
        subscribe(MQTT_SET_SOLAR_TRACKING);
        subscribe(MQTT_SOLAR_PRODUCTION_WATTS);
        subscribe(MQTT_HOUSE_CONSUMPTION_WATTS);
        // Unified topics (broadcast + per-device config)
        subscribe(MQTT_GET_ALL);
        subscribe(ht(TOPIC_UNIFIED_HOSTNAME).c_str());
        subscribe(ht(TOPIC_UNIFIED_PZEM_ADDRESS).c_str());
        subscribe(ht(TOPIC_UNIFIED_PZEM_ACQ_RATE).c_str());
        subscribe(ht(TOPIC_UNIFIED_DBG_PZEM_STATE).c_str());
        subscribe(ht(TOPIC_UNIFIED_DBG_PZEM_RATE).c_str());
        subscribe(ht(TOPIC_UNIFIED_MQTT_SERVER).c_str());
        subscribe(ht(TOPIC_UNIFIED_MQTT_PORT).c_str());
        subscribe(ht(TOPIC_UNIFIED_RESTART).c_str());
        publishDebugStatus();
        publishPwm();
        return true;
    }
    return false;
}

void MqttHandler::loop() {
    if (m_pubsubClient) m_pubsubClient->loop();
    applySolarTracking();
}

void MqttHandler::subscribe(const char* topic) {
    if (m_pubsubClient && m_pubsubClient->connected()) {
        m_pubsubClient->subscribe(topic);
    }
}

void MqttHandler::publish(const char* topic, const char* message) {
    if (m_pubsubClient && m_pubsubClient->connected()) {
        m_pubsubClient->publish(topic, message);
    }
}

void MqttHandler::publish(const char* topic, const char* message, bool retain) {
    if (m_pubsubClient && m_pubsubClient->connected()) {
        m_pubsubClient->publish(topic, message, retain);
    }
}

bool MqttHandler::isConnected() const {
    return m_pubsubClient && m_pubsubClient->connected();
}

void MqttHandler::getTimestamp(char* buf, size_t len) {
    time_t now = time(nullptr);
    if (now < 24 * 3600) {
        strncpy(buf, "unsynced", len);
        return;
    }
    struct tm t;
    localtime_r(&now, &t);
    strftime(buf, len, "%Y-%m-%dT%H:%M:%SZ", &t);
}

//-- State Publishing --//

void MqttHandler::publishState(const char* state, int cpp_max, int cpp_min, bool charging_enabled) {
    if (!isConnected()) return;
    if (m_debug_enabled && !(m_debug_flags & DBG_DETAILS)) return;
    char ts[24];
    getTimestamp(ts, sizeof(ts));
    char sz_msg[160];
    snprintf(sz_msg, sizeof(sz_msg),
        "{\"ts\":\"%s\",\"state\":\"%s\",\"CPP_max\":%d,\"CPP_min\":%d,\"charging_enabled\":%s,\"charging\":\"%s\",\"solar_tracking\":\"%s\"}",
        ts, state, cpp_max, cpp_min,
        charging_enabled ? "true" : "false",
        (state[0] == 'C') ? "on" : "off",
        m_solar_tracking ? "on" : "off");
    publish(MQTT_STATE_DETAILS, sz_msg);
}

void MqttHandler::publishTransition(char prev_state, char new_state) {
    if (!isConnected()) return;
    if (m_debug_enabled && !(m_debug_flags & DBG_CHANGE)) return;
    char ts[24];
    getTimestamp(ts, sizeof(ts));
    char sz_msg[60];
    snprintf(sz_msg, sizeof(sz_msg),
        "{\"ts\":\"%s\",\"from\":\"%c\",\"to\":\"%c\"}",
        ts, prev_state, new_state);
    publish(MQTT_STATE_CHANGE, sz_msg);
}

void MqttHandler::publishSpeed(int pwm) {
    if (!isConnected()) return;
    if (pwm == m_last_pwm) return;  // Only publish on change
    m_last_pwm = pwm;
    char ts[24];
    getTimestamp(ts, sizeof(ts));
    char sz_msg[80];
    snprintf(sz_msg, sizeof(sz_msg),
        "{\"ts\":\"%s\",\"pwm\":%d,\"W\":%.1f}",
        ts, pwm, m_last_power_w);
    publish(MQTT_SPEED, sz_msg, true);
}

void MqttHandler::publishPwm() {
    if (!isConnected()) return;
    int setpoint = m_evseController ? m_evseController->getChargeSpeed() : m_last_pwm;
    int current_pwm = 0;
    if (m_evseController) {
        int s = m_evseController->getCurrentState();
        bool enabled = m_evseController->isChargingEnabled();
        if (s == STATE_C && enabled) current_pwm = setpoint;
    }
    char ts[24];
    getTimestamp(ts, sizeof(ts));
    char sz_msg[80];
    snprintf(sz_msg, sizeof(sz_msg),
        "{\"ts\":\"%s\",\"current\":%d,\"setpoint\":%d}",
        ts, current_pwm, setpoint);
    publish(MQTT_STATE_PWM, sz_msg, true);
}

//-- Telemetry --//

void MqttHandler::publishTelemetry(float voltage, float current, float power, float energy, float frequency, float pf) {
    if (!isConnected()) return;
    m_last_power_w = power;
    char ts[24];
    getTimestamp(ts, sizeof(ts));
    // Always cache last reading for home/get/debug/all responses
    snprintf(m_last_pzem_json, sizeof(m_last_pzem_json),
        "{\"ts\":\"%s\",\"device\":\"%s\",\"V\":%.1f,\"I\":%.2f,\"W\":%.1f,\"kWh\":%.3f,\"Hz\":%.1f,\"PF\":%.2f}",
        ts, m_hostname, voltage, current, power, energy, frequency, pf);
    // Apply debug gate + publish rate
    if (m_debug_enabled && !(m_debug_flags & DBG_PZEM)) return;
    static unsigned long last_pub = 0;
    unsigned long now = millis();
    if (now - last_pub < (unsigned long)m_pzem_pub_rate * 1000UL) return;
    last_pub = now;
    publish(MQTT_TELEMETRY, m_last_pzem_json);
}

//-- Debug --//

void MqttHandler::publishDebugStatus() {
    if (!isConnected()) return;

    // Build flags array with all flags listed, each with name, value and state
    // Format: [{"name":"STATE","value":1,"active":true}, ...]
    static const struct { uint8_t bit; const char* name; } k_flags[] = {
        { DBG_DETAILS, "DETAILS" },
        { DBG_CHANGE,  "CHANGE"  },
        { DBG_PZEM,    "PZEM"    },
        { DBG_COMM,    "COMM"    },
        { DBG_TIME,    "TIME"    },
    };
    char flags_arr[280];
    flags_arr[0] = '['; flags_arr[1] = '\0';
    for (int i = 0; i < 5; i++) {
        char entry[48];
        snprintf(entry, sizeof(entry),
            "%s{\"name\":\"%s\",\"value\":%d,\"active\":%s}",
            i > 0 ? "," : "",
            k_flags[i].name,
            k_flags[i].bit,
            (m_debug_flags & k_flags[i].bit) ? "true" : "false");
        strcat(flags_arr, entry);
    }
    strcat(flags_arr, "]");

    char sz_msg[320];
    snprintf(sz_msg, sizeof(sz_msg),
        "{\"debug\":\"%s\",\"flags\":%d,\"flag_details\":%s}",
        m_debug_enabled ? "on" : "off", m_debug_flags, flags_arr);
    publish(MQTT_STATE_DEBUG, sz_msg, true);
}

void MqttHandler::setDebugEnabled(bool enabled) {
    m_debug_enabled = enabled;
    Preferences prefs;
    prefs.begin("zozo", false);
    prefs.putBool("dbg_en", enabled);
    prefs.end();
    publishDebugStatus();
}

void MqttHandler::setDebugFlags(uint8_t flags) {
    m_debug_flags = flags;
    Preferences prefs;
    prefs.begin("zozo", false);
    prefs.putUChar("dbg_flags", flags);
    prefs.end();
    publishDebugStatus();
}

void MqttHandler::setPzemOk(bool ok) {
    m_pzem_ok = ok;
}

void MqttHandler::publishComm() {
    if (!isConnected()) return;
    if (m_debug_enabled && !(m_debug_flags & DBG_COMM)) return;
    // Build status strings
    char sz_ip[48];
    snprintf(sz_ip, sizeof(sz_ip), "%s (%s)",
        WiFi.localIP().toString().c_str(),
        (WiFi.status() == WL_CONNECTED) ? "connected" : "disconnected");

    char sz_mqtt[80];
    snprintf(sz_mqtt, sizeof(sz_mqtt), "%s:%d (%s)",
        m_mqtt_server, m_mqtt_port,
        isConnected() ? "connected" : "disconnected");

    char sz_pzem[20];
    snprintf(sz_pzem, sizeof(sz_pzem), "0x%02X (%s)",
        m_pzem_addr, m_pzem_ok ? "ok" : "failed");

    // Deduplicate: skip if nothing changed
    static char last_payload[160] = "";
    char sz_msg[160];
    snprintf(sz_msg, sizeof(sz_msg),
        "{\"ip\":\"%s\",\"mqtt\":\"%s\",\"pzem_addr\":\"%s\"}",
        sz_ip, sz_mqtt, sz_pzem);
    if (strcmp(sz_msg, last_payload) == 0) return;
    strncpy(last_payload, sz_msg, sizeof(last_payload) - 1);

    // Add timestamp and device field
    char ts[24];
    getTimestamp(ts, sizeof(ts));
    char sz_full[220];
    snprintf(sz_full, sizeof(sz_full),
        "{\"ts\":\"%s\",\"device\":\"%s\",\"ip\":\"%s\",\"wifi\":\"%s\",\"mqtt\":\"%s\",\"pzem_addr\":\"%s\"}",
        ts, m_hostname, sz_ip, WIFI_SSID, sz_mqtt, sz_pzem);
    publish(MQTT_STATE_COMM, sz_full, true);
}

void MqttHandler::publishTime() {
    static unsigned long last = 0;
    static char sync_ts[24] = "";
    if (millis() - last < 60000) return;
    last = millis();
    if (!isConnected()) return;
    if (m_debug_enabled && !(m_debug_flags & DBG_TIME)) return;
    char ts[24];
    getTimestamp(ts, sizeof(ts));
    if (strcmp(ts, "unsynced") == 0) return;
    if (sync_ts[0] == '\0') strncpy(sync_ts, ts, sizeof(sync_ts));
    char sz_msg[120];
    snprintf(sz_msg, sizeof(sz_msg),
        "{\"ts\":\"%s\",\"device\":\"%s\",\"synced\":true,\"uptime_s\":%lu,\"sync_ts\":\"%s\"}",
        ts, m_hostname, millis() / 1000UL, sync_ts);
    publish(MQTT_STATE_TIME, sz_msg, true);
}

void MqttHandler::publishAllDebug() {
    if (!isConnected()) return;
    // Publish last cached state/pzem (no new sensor read)
    if (m_last_pzem_json[0] != '\0') {
        publish(MQTT_TELEMETRY, m_last_pzem_json);
    }
    // Publish state/comm (bypass dedup + debug gate)
    {
        char sz_ip[48];
        snprintf(sz_ip, sizeof(sz_ip), "%s (%s)",
            WiFi.localIP().toString().c_str(),
            (WiFi.status() == WL_CONNECTED) ? "connected" : "disconnected");
        char sz_mqtt[80];
        snprintf(sz_mqtt, sizeof(sz_mqtt), "%s:%d (%s)",
            m_mqtt_server, m_mqtt_port, "connected");
        char sz_pzem[20];
        snprintf(sz_pzem, sizeof(sz_pzem), "0x%02X (%s)",
            m_pzem_addr, m_pzem_ok ? "ok" : "failed");
        char ts[24]; getTimestamp(ts, sizeof(ts));
        char sz_full[220];
        snprintf(sz_full, sizeof(sz_full),
            "{\"ts\":\"%s\",\"device\":\"%s\",\"ip\":\"%s\",\"wifi\":\"%s\",\"mqtt\":\"%s\",\"pzem_addr\":\"%s\"}",
            ts, m_hostname, sz_ip, WIFI_SSID, sz_mqtt, sz_pzem);
        publish(MQTT_STATE_COMM, sz_full, true);
    }
    // Publish state/time (bypass debug gate)
    {
        char ts[24]; getTimestamp(ts, sizeof(ts));
        if (strcmp(ts, "unsynced") != 0) {
            char sz_msg[120];
            snprintf(sz_msg, sizeof(sz_msg),
                "{\"ts\":\"%s\",\"device\":\"%s\",\"uptime_s\":%lu}",
                ts, m_hostname, millis() / 1000UL);
            publish(MQTT_STATE_TIME, sz_msg, true);
        }
    }
    publishDebugStatus();
}

//-- Solar Tracking Control Loop --//

void MqttHandler::applySolarTracking() {
    if (!m_solar_tracking) return;
    if (!m_evseController) return;

    unsigned long now = millis();
    if (now - m_solar_last_step_ms < SOLAR_LOOP_MS) return;
    m_solar_last_step_ms = now;

    float evse_power = m_evseController->getPower();
    float diff = m_solar_watts - evse_power;
    int pwm_current = m_evseController->getChargeSpeed();
    int pwm_next = pwm_current;
    const char* action = "hold";

    if (diff > SOLAR_DEADBAND_W) {
        pwm_next -= SOLAR_STEP_DOWN;   // surplus: ramp up current (lower PWM)
        action = "down";
    } else if (diff < -SOLAR_DEADBAND_W) {
        pwm_next += SOLAR_STEP_UP;     // deficit: back off current (higher PWM)
        action = "up";
    }

    if (pwm_next < SOLAR_PWM_MIN) pwm_next = SOLAR_PWM_MIN;
    if (pwm_next > SOLAR_PWM_MAX) pwm_next = SOLAR_PWM_MAX;

    if (pwm_next != pwm_current) {
        m_evseController->setChargeSpeed(pwm_next);
    }

    // Publish debug telemetry every cycle
    if (isConnected()) {
        char ts[24];
        getTimestamp(ts, sizeof(ts));
        char sz_msg[160];
        snprintf(sz_msg, sizeof(sz_msg),
            "{\"ts\":\"%s\",\"solar_w\":%.1f,\"evse_w\":%.1f,\"diff_w\":%.1f,\"pwm\":%d,\"pwm_next\":%d,\"action\":\"%s\"}",
            ts, m_solar_watts, evse_power, diff, pwm_current, pwm_next, action);
        publish(MQTT_STATE_SOLAR_TRACKING, sz_msg);
    }
}

//-- Message Handling --//

void MqttHandler::handleMessage(char* topic, uint8_t* payload, unsigned int length) {
    payload[length] = '\0';
    char ts[24];
    getTimestamp(ts, sizeof(ts));
    char sz_conf[120];

    if (strcmp(topic, MQTT_SET_CHARGE_RATE) == 0) {
        int i_pwm = atoi((char*)payload);
        if (i_pwm < 0) i_pwm = 0;
        if (i_pwm > 255) i_pwm = 255;
        if (m_evseController) m_evseController->setChargeSpeed(i_pwm);
        publishPwm();
        snprintf(sz_conf, sizeof(sz_conf),
            "{\"cmd\":\"charge_rate\",\"result\":\"ok\",\"value\":%d,\"ts\":\"%s\"}", i_pwm, ts);
        publish(MQTT_SET_CHARGE_RATE_STATUS, sz_conf);
    }

    if (strcmp(topic, MQTT_SET_DELAY) == 0) {
        bool enable = (strcmp((char*)payload, "off") == 0);
        if (m_evseController) m_evseController->setChargingEnabled(enable);
        const char* state_str = "?";
        if (m_evseController) {
            int s = m_evseController->getCurrentState();
            if (s == STATE_A) state_str = "A";
            else if (s == STATE_B) state_str = "B";
            else if (s == STATE_C) state_str = "C";
            else if (s == STATE_SLEEPING) state_str = "S";
        }
        snprintf(sz_conf, sizeof(sz_conf),
            "{\"cmd\":\"delay\",\"result\":\"ok\",\"delay\":\"%s\",\"state\":\"%s\",\"ts\":\"%s\"}",
            enable ? "off" : "on", state_str, ts);
        publish(MQTT_SET_DELAY_STATUS, sz_conf);
    }

    if (strcmp(topic, MQTT_SET_DEBUG) == 0) {
        bool enabled = (strcmp((char*)payload, "on") == 0);
        setDebugEnabled(enabled);
        snprintf(sz_conf, sizeof(sz_conf),
            "{\"cmd\":\"debug\",\"result\":\"ok\",\"debug\":\"%s\",\"ts\":\"%s\"}",
            enabled ? "on" : "off", ts);
        publish(MQTT_SET_DEBUG_STATUS, sz_conf);
    }

    if (strcmp(topic, MQTT_SET_DEBUG_FLAGS) == 0) {
        uint8_t flags = (uint8_t)atoi((char*)payload);
        setDebugFlags(flags);
        snprintf(sz_conf, sizeof(sz_conf),
            "{\"cmd\":\"debug_flags\",\"result\":\"ok\",\"flags\":%d,\"ts\":\"%s\"}", flags, ts);
        publish(MQTT_SET_DEBUG_FLAGS_STATUS, sz_conf);
    }

    if (strcmp(topic, MQTT_GET_DEBUG) == 0) {
        publishDebugStatus();
    }

    if (strcmp(topic, MQTT_SET_SOLAR_TRACKING) == 0) {
        m_solar_tracking = (strcmp((char*)payload, "on") == 0);
        snprintf(sz_conf, sizeof(sz_conf),
            "{\"cmd\":\"solar_tracking\",\"result\":\"ok\",\"value\":\"%s\",\"ts\":\"%s\"}",
            m_solar_tracking ? "on" : "off", ts);
        publish(MQTT_SET_SOLAR_TRACKING_STATUS, sz_conf);
    }

    if (strcmp(topic, MQTT_SOLAR_PRODUCTION_WATTS) == 0) {
        m_solar_watts = atof((char*)payload);
    }

    if (strcmp(topic, MQTT_HOUSE_CONSUMPTION_WATTS) == 0) {
        m_house_watts = atof((char*)payload);
    }

    // ── Unified config commands ───────────────────────────────────────────────
    if (strcmp(topic, MQTT_GET_ALL) == 0) {
        // Defer to next loop() so a fresh PZEM read can happen first
        m_poll_requested = true;
        return;
    }
    if (strcmp(topic, ht(TOPIC_UNIFIED_HOSTNAME).c_str()) == 0) {
        strncpy(m_hostname, (char*)payload, sizeof(m_hostname) - 1);
        m_hostname[sizeof(m_hostname) - 1] = '\0';
        saveConfig();
        publishComm();
    }
    else if (strcmp(topic, ht(TOPIC_UNIFIED_PZEM_ADDRESS).c_str()) == 0) {
        uint8_t addr = (uint8_t)strtoul((char*)payload, nullptr, 0);
        if (addr >= 0x01 && addr <= 0x0F) {
            m_pzem_addr = addr;
            saveConfig();
            publishComm();   // reflects new address immediately in state/comm
        }
    }
    else if (strcmp(topic, ht(TOPIC_UNIFIED_PZEM_ACQ_RATE).c_str()) == 0) {
        uint16_t rate = (uint16_t)atoi((char*)payload);
        if (rate >= 5) { m_pzem_acq_rate = rate; saveConfig(); }
    }
    else if (strcmp(topic, ht(TOPIC_UNIFIED_DBG_PZEM_STATE).c_str()) == 0) {
        bool en = (strcmp((char*)payload, "on") == 0);
        // Map to DBG_PZEM flag
        if (en) m_debug_flags |=  DBG_PZEM;
        else    m_debug_flags &= ~DBG_PZEM;
        Preferences prefs; prefs.begin("zozo", false);
        prefs.putUChar("dbg_flags", m_debug_flags); prefs.end();
        publishDebugStatus();
    }
    else if (strcmp(topic, ht(TOPIC_UNIFIED_DBG_PZEM_RATE).c_str()) == 0) {
        uint16_t rate = (uint16_t)atoi((char*)payload);
        if (rate >= 5) { m_pzem_pub_rate = rate; saveConfig(); }
    }
    else if (strcmp(topic, ht(TOPIC_UNIFIED_MQTT_SERVER).c_str()) == 0) {
        strncpy(m_mqtt_server, (char*)payload, sizeof(m_mqtt_server) - 1);
        m_mqtt_server[sizeof(m_mqtt_server) - 1] = '\0';
        saveConfig();
        publishComm();
    }
    else if (strcmp(topic, ht(TOPIC_UNIFIED_MQTT_PORT).c_str()) == 0) {
        uint16_t port = (uint16_t)atoi((char*)payload);
        if (port > 0) { m_mqtt_port = port; saveConfig(); publishComm(); }
    }
    else if (strcmp(topic, ht(TOPIC_UNIFIED_RESTART).c_str()) == 0) {
        delay(300);
        ESP.restart();
    }
}
