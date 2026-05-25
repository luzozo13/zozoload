#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include <PZEM004Tv30.h>
#include <time.h>
#include "wifi_credentials.h"
#include "mqtt_config.h"

// ── EEPROM layout ─────────────────────────────────────────────────────────────
// Bump EEPROM_SCHEMA_VERSION when field layout changes; triggers auto-reset on mismatch.
#define EEPROM_SIZE                   512
#define EEPROM_SCHEMA_VERSION         7   // bumped: removed debugFlags
#define EEPROM_SCHEMA_ADDR            500   // 1 byte, at end of space
#define EEPROM_HOSTNAME_ADDR          0     // 64 bytes
#define EEPROM_MQTT_SERVER_ADDR       66    // 64 bytes
#define EEPROM_MQTT_PORT_ADDR         130   // 2 bytes
#define EEPROM_PZEM_ACQ_RATE_ADDR     133   // 2 bytes
#define EEPROM_DBG_ENABLED_ADDR       135   // 1 byte

// ── Settings (struct defaults = factory values after auto-reset) ──────────────
#ifndef DEFAULT_HOSTNAME
  #define DEFAULT_HOSTNAME "sonoff-mini-pzem"
#endif

struct Settings {
    char     hostname[64]   = DEFAULT_HOSTNAME;
    char     mqttServer[64] = MQTT_SERVER;
    uint16_t mqttPort       = MQTT_PORT;
    uint16_t pzemAcqRate    = 30;    // seconds: how often the sensor is polled
    bool     debugEnabled   = true;  // master debug on/off (gates state/pzem, state/comm, state/time)
} settings;

// ── PZEM cache ────────────────────────────────────────────────────────────────
struct PzemCache {
    float voltage   = NAN;
    float current   = NAN;
    float power     = NAN;
    float energy    = NAN;
    float frequency = NAN;
    float pf        = NAN;
    bool  valid     = false;
} pzemCache;

// ── Globals ───────────────────────────────────────────────────────────────────
WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);
PZEM004Tv30  pzem((Stream&)Serial, 0xF8);  // placeholder; properly init'd in setup()

bool          pzemOk          = false;
unsigned long lastAcquisition = 0;
unsigned long lastTimePublish = 0;
unsigned long eepromWriteTime = 0;
bool          eepromPending   = false;
unsigned long lastLEDBlink    = 0;
bool          ledState        = HIGH;

// ── Helper: build topic with hostname prefix ──────────────────────────────────
String ht(const char* suffix) { return String(settings.hostname) + suffix; }

// ── Timestamp helper (NTP-synced, falls back to "unsynced") ──────────────────
void getTimestamp(char* buf, size_t len) {
    time_t now = time(nullptr);
    if (now < 24UL * 3600UL) { strncpy(buf, "unsynced", len); return; }
    struct tm* t = localtime(&now);
    strftime(buf, len, "%Y-%m-%dT%H:%M:%S", t);
}

// ── EEPROM ────────────────────────────────────────────────────────────────────
void writeEEPROM() {
    EEPROM.begin(EEPROM_SIZE);
    EEPROM.put(EEPROM_MQTT_SERVER_ADDR,      settings.mqttServer);
    EEPROM.put(EEPROM_MQTT_PORT_ADDR,        settings.mqttPort);
    EEPROM.put(EEPROM_PZEM_ACQ_RATE_ADDR,    settings.pzemAcqRate);
    EEPROM.put(EEPROM_DBG_ENABLED_ADDR,      (uint8_t)(settings.debugEnabled ? 1 : 0));
    EEPROM.put(EEPROM_SCHEMA_ADDR,           (uint8_t)EEPROM_SCHEMA_VERSION);
    EEPROM.commit();
    EEPROM.end();
    yield();
}

void resetToDefaults() {
    Settings defaults;   // stack-allocated with all struct defaults
    settings = defaults;
    writeEEPROM();
}

void readEEPROM() {
#ifdef FORCE_EEPROM_RESET
    resetToDefaults();   // compile-time flag: overwrite EEPROM with build defaults
    return;
#endif
    EEPROM.begin(EEPROM_SIZE);
    uint8_t schema = 0;
    EEPROM.get(EEPROM_SCHEMA_ADDR, schema);
    EEPROM.end();
    if (schema != EEPROM_SCHEMA_VERSION) {
        resetToDefaults();   // auto-reset: new firmware or blank EEPROM
        return;
    }
    EEPROM.begin(EEPROM_SIZE);
    EEPROM.get(EEPROM_MQTT_SERVER_ADDR,      settings.mqttServer);
    settings.mqttServer[63] = '\0';
    EEPROM.get(EEPROM_MQTT_PORT_ADDR,        settings.mqttPort);
    EEPROM.get(EEPROM_PZEM_ACQ_RATE_ADDR,    settings.pzemAcqRate);
    uint8_t dbgEn = 1;
    EEPROM.get(EEPROM_DBG_ENABLED_ADDR,      dbgEn);
    settings.debugEnabled = (dbgEn != 0);
    EEPROM.end();
}

void scheduleEEPROMWrite() {
    eepromPending   = true;
    eepromWriteTime = millis();
}

// ── PZEM ─────────────────────────────────────────────────────────────────────
void acquirePZEM() {
    pzemCache.voltage   = pzem.voltage();
    pzemCache.current   = pzem.current();
    pzemCache.power     = pzem.power();
    pzemCache.energy    = pzem.energy();
    pzemCache.frequency = pzem.frequency();
    pzemCache.pf        = pzem.pf();
    pzemCache.valid     = !isnan(pzemCache.voltage);
    pzemOk              = pzemCache.valid;
}

// ── Publishers ────────────────────────────────────────────────────────────────
void publishStatePzem() {
    if (!mqttClient.connected()) return;
    if (!settings.debugEnabled) return;
    char ts[24]; getTimestamp(ts, sizeof(ts));
    char buf[200];
    if (pzemCache.valid) {
        snprintf(buf, sizeof(buf),
            "{\"ts\":\"%s\",\"device\":\"%s\",\"V\":%.1f,\"I\":%.3f,\"W\":%.1f,\"kWh\":%.3f,\"Hz\":%.1f,\"PF\":%.2f}",
            ts, settings.hostname,
            pzemCache.voltage, pzemCache.current, pzemCache.power,
            pzemCache.energy,  pzemCache.frequency, pzemCache.pf);
    } else {
        snprintf(buf, sizeof(buf),
            "{\"ts\":\"%s\",\"device\":\"%s\",\"error\":\"pzem_read_failed\"}", ts, settings.hostname);
    }
    mqttClient.publish(ht(TOPIC_STATE_PZEM).c_str(), buf, true);   // retained
}

void publishStateComm() {
    if (!mqttClient.connected()) return;
    if (!settings.debugEnabled) return;
    char ts[24]; getTimestamp(ts, sizeof(ts));
    char mqtt_str[80];
    snprintf(mqtt_str, sizeof(mqtt_str), "%s:%u", settings.mqttServer, settings.mqttPort);
    char buf[270];
    snprintf(buf, sizeof(buf),
        "{\"ts\":\"%s\",\"device\":\"%s\",\"ip\":\"%s\",\"wifi\":\"%s\",\"mqtt\":\"%s\",\"pzem\":\"%s\"}",
        ts, settings.hostname, WiFi.localIP().toString().c_str(),
        WIFI_SSID, mqtt_str, pzemOk ? "ok" : "fail");
    mqttClient.publish(ht(TOPIC_STATE_COMM).c_str(), buf, true);   // retained
}

void publishStateTime() {
    if (!mqttClient.connected()) return;
    if (!settings.debugEnabled) return;
    char ts[24]; getTimestamp(ts, sizeof(ts));
    char buf[130];
    snprintf(buf, sizeof(buf),
        "{\"ts\":\"%s\",\"device\":\"%s\",\"uptime_s\":%lu}", ts, settings.hostname, millis() / 1000UL);
    mqttClient.publish(ht(TOPIC_STATE_TIME).c_str(), buf, true);   // retained
}

void publishPower() {
    if (!mqttClient.connected() || !pzemCache.valid) return;
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f", pzemCache.power);
    mqttClient.publish(TOPIC_PUBLISH_POWER, buf, true);   // retained — absolute topic, no hostname prefix
}

void publishDebugStatus() {
    if (!mqttClient.connected()) return;
    char buf[32];
    snprintf(buf, sizeof(buf), "{\"debug\":\"%s\"}", settings.debugEnabled ? "on" : "off");
    mqttClient.publish(ht(TOPIC_STATE_DEBUG).c_str(), buf, true);   // retained
}

// Respond to home/get/debug/all — republish last stored values, no new sensor read
void publishAllDebug() {
    // Temporarily force debug on so all state topics publish regardless of gate
    bool savedEnabled = settings.debugEnabled;
    settings.debugEnabled = true;
    publishStatePzem();
    publishPower();
    publishStateComm();
    publishStateTime();
    settings.debugEnabled = savedEnabled;
    publishDebugStatus();
}

// ── MQTT message handler ──────────────────────────────────────────────────────
void handleMQTTMessage(char* topic, byte* payload, unsigned int length) {
    char msg[length + 1];
    memcpy(msg, payload, length);
    msg[length] = '\0';

    // Broadcast: all devices respond regardless of debug state
    if (strcmp(topic, MQTT_GET_ALL) == 0) {
        publishAllDebug();
        return;
    }
    if (strcmp(topic, ht(TOPIC_SET_PZEM_ACQ_RATE).c_str()) == 0) {
        uint16_t rate = (uint16_t)atoi(msg);
        if (rate >= 1) {
            settings.pzemAcqRate = rate;
            scheduleEEPROMWrite();
        }
    }
    else if (strcmp(topic, ht(TOPIC_SET_MQTT_SERVER).c_str()) == 0) {
        strncpy(settings.mqttServer, msg, sizeof(settings.mqttServer) - 1);
        settings.mqttServer[sizeof(settings.mqttServer) - 1] = '\0';
        scheduleEEPROMWrite();
        mqttClient.publish(ht(TOPIC_OTA_STATUS).c_str(), "mqtt server saved; restart to apply");
    }
    else if (strcmp(topic, ht(TOPIC_SET_MQTT_PORT).c_str()) == 0) {
        settings.mqttPort = (uint16_t)atoi(msg);
        scheduleEEPROMWrite();
        mqttClient.publish(ht(TOPIC_OTA_STATUS).c_str(), "mqtt port saved; restart to apply");
    }
    else if (strcmp(topic, ht(TOPIC_SET_DEBUG).c_str()) == 0) {
        settings.debugEnabled = (strcmp(msg, "on") == 0);
        scheduleEEPROMWrite();
        publishDebugStatus();
    }
    else if (strcmp(topic, ht(TOPIC_SET_RESTART).c_str()) == 0) {
        mqttClient.publish(ht(TOPIC_OTA_STATUS).c_str(), "restarting...");
        delay(300);
        ESP.restart();
    }
}

// ── MQTT reconnect ────────────────────────────────────────────────────────────
void reconnectMQTT() {
    if (mqttClient.connected()) return;
    static unsigned long lastAttempt = 0;
    if (millis() - lastAttempt < 5000UL) return;
    lastAttempt = millis();
    if (mqttClient.connect(settings.hostname)) {
        mqttClient.subscribe(MQTT_GET_ALL);
        mqttClient.subscribe(ht(TOPIC_SET_PZEM_ACQ_RATE).c_str());
        mqttClient.subscribe(ht(TOPIC_SET_MQTT_SERVER).c_str());
        mqttClient.subscribe(ht(TOPIC_SET_MQTT_PORT).c_str());
        mqttClient.subscribe(ht(TOPIC_SET_DEBUG).c_str());
        mqttClient.subscribe(ht(TOPIC_SET_RESTART).c_str());
        publishStateComm();
        publishDebugStatus();
    }
}

// ── setup ─────────────────────────────────────────────────────────────────────
void setup() {
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);

    readEEPROM();

    // Compile-time debug overrides — applied after EEPROM read, persisted to EEPROM
#if defined(FORCE_DEBUG_ENABLED)
    settings.debugEnabled = true;  scheduleEEPROMWrite();
#elif defined(FORCE_DEBUG_DISABLED)
    settings.debugEnabled = false; scheduleEEPROMWrite();
#endif
    pzem = PZEM004Tv30(Serial);   // Serial.begin(9600) called here

    // WiFi + NTP (both non-blocking)
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
    tzset();

    ArduinoOTA.setHostname(settings.hostname);
    ArduinoOTA.onStart([]() {
        mqttClient.publish(ht(TOPIC_OTA_STATUS).c_str(), "OTA starting...");
    });
    ArduinoOTA.onEnd([]() {
        mqttClient.publish(ht(TOPIC_OTA_STATUS).c_str(), "OTA complete");
    });
    ArduinoOTA.onProgress([](unsigned int, unsigned int) {});
    ArduinoOTA.onError([](ota_error_t) {});
    ArduinoOTA.begin();

    mqttClient.setServer(settings.mqttServer, settings.mqttPort);
    mqttClient.setCallback(handleMQTTMessage);
}

// ── loop ──────────────────────────────────────────────────────────────────────
void loop() {
    ArduinoOTA.handle();

    // Deferred EEPROM write: 1 s after last change
    if (eepromPending && (millis() - eepromWriteTime > 1000UL)) {
        eepromPending = false;
        writeEEPROM();
    }

    // LED heartbeat: 1 s = MQTT OK, 200 ms = no MQTT
    unsigned long blinkInterval = mqttClient.connected() ? 1000UL : 200UL;
    if (millis() - lastLEDBlink > blinkInterval) {
        lastLEDBlink = millis();
        ledState = !ledState;
        digitalWrite(13, ledState);
    }

    if (WiFi.status() == WL_CONNECTED) {
        reconnectMQTT();
        mqttClient.loop();

        unsigned long now = millis();

        // ── Regular PZEM acquisition ───────────────────────────────────────────
        if (now - lastAcquisition >= (unsigned long)settings.pzemAcqRate * 1000UL) {
            lastAcquisition = now;
            bool prevOk = pzemOk;
            acquirePZEM();
            publishStatePzem();
            publishPower();
            if (pzemOk != prevOk) publishStateComm();
        }

        // state/time: always published every 60 s (retained)
        if (now - lastTimePublish >= 60000UL) {
            lastTimePublish = now;
            publishStateTime();
        }
    }

    delay(100);
}
