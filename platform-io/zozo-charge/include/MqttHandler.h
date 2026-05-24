//-- MQTT Handler Class - Header --//
#pragma once

#include <PubSubClient.h>
#include <Preferences.h>

class EVSEController;

class MqttHandler {
private:
    PubSubClient* m_pubsubClient;
    EVSEController* m_evseController;
    uint8_t m_debug_flags = 0;
    bool m_debug_enabled = false;
    float m_solar_watts = 0.0f;
    float m_house_watts = 0.0f;
    int m_last_pwm = 0;
    float m_last_power_w = 0.0f;
    bool m_pzem_ok = false;

    void getTimestamp(char* buf, size_t len);
    void publishDebugStatus();

public:
    MqttHandler(PubSubClient& client);

    void setup(const char* server, int port);
    void setEVSEController(EVSEController& controller);
    bool reconnect(const char* clientId = "zozo-charge");
    void loop();
    void loadFromNVS();

    void publish(const char* topic, const char* message);
    void publish(const char* topic, const char* message, bool retain);
    void subscribe(const char* topic);
    bool isConnected() const;

    void publishState(const char* state, int cpp_max, int cpp_min, bool charging_enabled);
    void publishTransition(char prev_state, char new_state);
    void publishSpeed(int pwm);
    void publishPwm();
    void publishTelemetry(float voltage, float current, float power, float energy, float frequency, float pf);
    void publishComm();
    void setPzemOk(bool ok);
    void publishTime();

    void setDebugEnabled(bool enabled);
    void setDebugFlags(uint8_t flags);
    uint8_t getDebugFlags() const { return m_debug_flags; }
    bool isDebugEnabled() const { return m_debug_enabled; }
    float getSolarWatts() const { return m_solar_watts; }
    float getHouseWatts() const { return m_house_watts; }

    void handleMessage(char* topic, uint8_t* payload, unsigned int length);
};

