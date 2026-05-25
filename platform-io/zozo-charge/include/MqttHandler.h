//-- MQTT Handler Class - Header --//
#pragma once

#include <PubSubClient.h>
#include <Preferences.h>

class EVSEController;

class MqttHandler {
private:
    PubSubClient* m_pubsubClient;
    EVSEController* m_evseController;

    //-- Existing debug state --//
    uint8_t m_debug_flags = 0;
    bool m_debug_enabled = false;
    float m_solar_watts = 0.0f;
    float m_house_watts = 0.0f;
    int m_last_pwm = 0;
    float m_last_power_w = 0.0f;
    bool m_pzem_ok = false;

    //-- Unified config (persisted in NVS) --//
    char     m_hostname[64];
    uint8_t  m_pzem_addr;
    uint16_t m_pzem_acq_rate;   // seconds: PZEM sensor read interval
    uint16_t m_pzem_pub_rate;   // seconds: state/pzem publish interval
    char     m_mqtt_server[64];
    uint16_t m_mqtt_port;

    //-- Cached last state/pzem payload (for home/get/debug/all response) --//
    char m_last_pzem_json[200];

    //-- Flag: main loop should do a fresh PZEM read then call publishAllDebug() --//
    bool m_poll_requested = false;

    //-- Solar tracking --//
    bool m_solar_tracking = true;           // Default ON: active immediately on boot
    unsigned long m_solar_last_step_ms = 0; // Timestamp of last control step

    void getTimestamp(char* buf, size_t len);
    void publishDebugStatus();
    String ht(const char* suffix);   // build topic: m_hostname + suffix

    void saveConfig();               // write unified config fields to NVS
    void applySolarTracking();       // closed-loop solar control step

public:
    MqttHandler(PubSubClient& client);

    void setup(const char* server, int port);
    void setup();                    // overload: uses NVS-loaded server/port
    void setEVSEController(EVSEController& controller);
    bool reconnect();
    void loop();
    void loadFromNVS();

    void publish(const char* topic, const char* message);
    void publish(const char* topic, const char* message, bool retain);
    void subscribe(const char* topic);
    bool isConnected() const;

    //-- Existing state publishers --//
    void publishState(const char* state, int cpp_max, int cpp_min, bool charging_enabled);
    void publishTransition(char prev_state, char new_state);
    void publishSpeed(int pwm);
    void publishPwm();
    void publishTelemetry(float voltage, float current, float power, float energy, float frequency, float pf);
    void publishComm();
    void setPzemOk(bool ok);
    void publishTime();

    //-- Unified: respond to home/get/debug/all --//
    void publishAllDebug();
    bool isPollRequested() const { return m_poll_requested; }
    void clearPollRequest()      { m_poll_requested = false; }

    //-- Debug flags --//
    void setDebugEnabled(bool enabled);
    void setDebugFlags(uint8_t flags);
    uint8_t getDebugFlags() const { return m_debug_flags; }
    bool isDebugEnabled() const { return m_debug_enabled; }

    //-- External data --//
    float getSolarWatts() const { return m_solar_watts; }
    float getHouseWatts() const { return m_house_watts; }
    bool isSolarTracking() const { return m_solar_tracking; }

    //-- Unified config getters (used by main.cpp) --//
    const char* getHostname()    const { return m_hostname; }
    uint8_t  getPzemAddr()       const { return m_pzem_addr; }
    uint16_t getPzemAcqRate()    const { return m_pzem_acq_rate; }
    uint16_t getPzemPubRate()    const { return m_pzem_pub_rate; }
    const char* getMqttServer()  const { return m_mqtt_server; }
    uint16_t getMqttPort()       const { return m_mqtt_port; }

    void handleMessage(char* topic, uint8_t* payload, unsigned int length);
};

