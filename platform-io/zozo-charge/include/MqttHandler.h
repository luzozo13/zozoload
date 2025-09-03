//-- MQTT Handler Class - Header --//
// Minimalist MQTT communication handler
// Provides clean interface for MQTT operations

#pragma once

#include <PubSubClient.h>

// Forward declaration to avoid circular dependency
class EVSEController;

class MqttHandler {
private:
    PubSubClient* m_pubsubClient;  // Pointer to MQTT client
    EVSEController* m_evseController; // Pointer to EVSE controller for callbacks
    
public:
    // Constructor - takes reference to existing PubSubClient
    MqttHandler(PubSubClient& client);
    
    // MQTT Setup and Management
    void setup(const char* server, int port);
    void setEVSEController(EVSEController& controller);
    bool reconnect(const char* clientId = "zozo-charge");
    void loop();
    
    // Core MQTT operations
    void publish(const char* topic, const char* message);
    void publish(const char* topic, const char* message, bool retain);
    void subscribe(const char* topic);
    
    // Status check
    bool isConnected() const;
    
    // Utility methods for common message types
    void publishState(const char* state, int cpp_max, int cpp_min);
    void publishTransition(char prev_state, char new_state);
    void publishSpeed(const char* speed_msg);
    void publishSpeed(int speed_value);
    
    // Internal callback handler (to be called by global callback)
    void handleMessage(char* topic, uint8_t* payload, unsigned int length);
};
