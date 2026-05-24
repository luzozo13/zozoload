//-- MQTT Handler Implementation --//

#include "MqttHandler.h"
#include "EVSEController.h"
#include "params.h"
#include "mqtt_config.h"
#include <Arduino.h>
#include <WiFi.h>
#include <stdio.h>
#include <time.h>

MqttHandler::MqttHandler(PubSubClient& client) : m_pubsubClient(&client), m_evseController(nullptr) {}

void MqttHandler::setup(const char* server, int port) {
    if (m_pubsubClient) {
        m_pubsubClient->setServer(server, port);
    }
}

void MqttHandler::setEVSEController(EVSEController& controller) {
    m_evseController = &controller;
}

void MqttHandler::loadFromNVS() {
    Preferences prefs;
    prefs.begin("zozo", false);
    uint8_t schema = prefs.getUChar("schema", 0);
    if (schema != NVS_SCHEMA_VERSION) {
        // New firmware version or first boot: reset to defaults
        m_debug_enabled = true;
        m_debug_flags   = DBG_DEFAULT_FLAGS;
        prefs.putBool("dbg_en",     m_debug_enabled);
        prefs.putUChar("dbg_flags", m_debug_flags);
        prefs.putUChar("schema",    NVS_SCHEMA_VERSION);
    } else {
        m_debug_flags   = prefs.getUChar("dbg_flags", DBG_DEFAULT_FLAGS);
        m_debug_enabled = prefs.getBool("dbg_en",     true);
    }
    prefs.end();
}

bool MqttHandler::reconnect(const char* clientId) {
    if (!m_pubsubClient) return false;
    if (m_pubsubClient->connect(clientId)) {
        subscribe(MQTT_SET_CHARGE_RATE);
        subscribe(MQTT_SET_DELAY);
        subscribe(MQTT_SET_DEBUG);
        subscribe(MQTT_SET_DEBUG_FLAGS);
        subscribe(MQTT_GET_DEBUG);
        subscribe(MQTT_SOLAR_PRODUCTION_WATTS);
        subscribe(MQTT_HOUSE_CONSUMPTION_WATTS);
        publishDebugStatus();
        publishPwm();
        return true;
    }
    return false;
}

void MqttHandler::loop() {
    if (m_pubsubClient) m_pubsubClient->loop();
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
    char sz_msg[120];
    snprintf(sz_msg, sizeof(sz_msg),
        "{\"ts\":\"%s\",\"state\":\"%s\",\"CPP_max\":%d,\"CPP_min\":%d,\"charging_enabled\":%s,\"charging\":\"%s\"}",
        ts, state, cpp_max, cpp_min,
        charging_enabled ? "true" : "false",
        (state[0] == 'C') ? "on" : "off");
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
    if (m_debug_enabled && !(m_debug_flags & DBG_PZEM)) return;
    m_last_power_w = power;
    char ts[24];
    getTimestamp(ts, sizeof(ts));
    char sz_msg[128];
    snprintf(sz_msg, sizeof(sz_msg),
        "{\"ts\":\"%s\",\"V\":%.1f,\"I\":%.2f,\"W\":%.1f,\"kWh\":%.3f,\"Hz\":%.1f,\"PF\":%.2f}",
        ts, voltage, current, power, energy, frequency, pf);
    publish(MQTT_TELEMETRY, sz_msg);
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

    char sz_mqtt[48];
    snprintf(sz_mqtt, sizeof(sz_mqtt), "%s:%d (%s)",
        MQTT_SERVER, MQTT_PORT,
        isConnected() ? "connected" : "disconnected");

    char sz_pzem[20];
    snprintf(sz_pzem, sizeof(sz_pzem), "0x%02X (%s)",
        PZEM_ADDR, m_pzem_ok ? "ok" : "failed");

    // Deduplicate: skip if nothing changed
    static char last_payload[160] = "";
    char sz_msg[160];
    snprintf(sz_msg, sizeof(sz_msg),
        "{\"ip\":\"%s\",\"mqtt\":\"%s\",\"pzem\":\"%s\"}",
        sz_ip, sz_mqtt, sz_pzem);
    if (strcmp(sz_msg, last_payload) == 0) return;
    strncpy(last_payload, sz_msg, sizeof(last_payload) - 1);

    // Add timestamp (of first occurrence of this status)
    char ts[24];
    getTimestamp(ts, sizeof(ts));
    char sz_full[200];
    snprintf(sz_full, sizeof(sz_full),
        "{\"ts\":\"%s\",\"ip\":\"%s\",\"mqtt\":\"%s\",\"pzem\":\"%s\"}",
        ts, sz_ip, sz_mqtt, sz_pzem);
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
    char sz_msg[100];
    snprintf(sz_msg, sizeof(sz_msg),
        "{\"ts\":\"%s\",\"synced\":true,\"uptime_s\":%lu,\"sync_ts\":\"%s\"}",
        ts, millis() / 1000UL, sync_ts);
    publish(MQTT_STATE_TIME, sz_msg, true);
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

    if (strcmp(topic, MQTT_SOLAR_PRODUCTION_WATTS) == 0) {
        m_solar_watts = atof((char*)payload);
    }

    if (strcmp(topic, MQTT_HOUSE_CONSUMPTION_WATTS) == 0) {
        m_house_watts = atof((char*)payload);
    }
}
