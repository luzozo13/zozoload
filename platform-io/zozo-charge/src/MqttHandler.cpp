//-- MQTT Handler Implementation --//
// Minimalist MQTT communication handler

#include "MqttHandler.h"
#include "params.h" // For EVSEController class
#include "mqtt_config.h"
#include <stdio.h>

//-- Constructor --//

MqttHandler::MqttHandler(PubSubClient& client) : m_pubsubClient(&client), m_evseController(nullptr) {
    // Store reference to existing MQTT client
}

//-- MQTT Setup and Management --//

void MqttHandler::setup(const char* server, int port) {
    if (m_pubsubClient) {
        m_pubsubClient->setServer(server, port);
        // Note: Callback must be set externally due to C++ limitations with member function pointers
    }
}

void MqttHandler::setEVSEController(EVSEController& controller) {
    m_evseController = &controller;
}

bool MqttHandler::reconnect(const char* clientId) {
    if (!m_pubsubClient) return false;
    
    if (m_pubsubClient->connect(clientId)) {
        // Subscribe to control topics
        subscribe(MQTT_SET_PWM);
        return true;
    }
    return false;
}

void MqttHandler::loop() {
    if (m_pubsubClient) {
        m_pubsubClient->loop();
    }
}

void MqttHandler::subscribe(const char* topic) {
    if (m_pubsubClient && m_pubsubClient->connected()) {
        m_pubsubClient->subscribe(topic);
    }
}

//-- Core MQTT Operations --//

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

//-- Utility Methods for EVSE-specific Messages --//

void MqttHandler::publishState(const char* state, int cpp_max, int cpp_min) {
    if (!isConnected()) return;
    
    // Compose JSON message
    char sz_msg[64];
    snprintf(sz_msg, sizeof(sz_msg),
        "{\"state\":\"%s\",\"CPP_max\":%d,\"CPP_min\":%d}",
        state, cpp_max, cpp_min);
    
    publish(MQTT_STATE, sz_msg);
}

void MqttHandler::publishTransition(char prev_state, char new_state) {
    if (!isConnected()) return;
    
    char sz_transition_msg[8];
    snprintf(sz_transition_msg, sizeof(sz_transition_msg), "%c->%c", prev_state, new_state);
    publish(MQTT_CHANGE, sz_transition_msg);
}

void MqttHandler::publishSpeed(const char* speed_msg) {
    if (!isConnected()) return;
    publish(MQTT_SPEED, speed_msg);
}

void MqttHandler::publishSpeed(int speed_value) {
    if (!isConnected()) return;
    
    char sz_speed_msg[8];
    snprintf(sz_speed_msg, sizeof(sz_speed_msg), "%d", speed_value);
    publish(MQTT_SPEED, sz_speed_msg);
}

//-- Message Handling --//

void MqttHandler::handleMessage(char* topic, uint8_t* payload, unsigned int length) {
    payload[length] = '\0'; // Null-terminate first
    
    if (strcmp(topic, MQTT_SET_PWM) == 0) {
        int i_pwm = atoi((char*)payload); // Convert payload to int
        if (i_pwm < 0) i_pwm = 0;
        if (i_pwm > 255) i_pwm = 255;
        
        // Update charge speed through controller if available
        if (m_evseController) {
            m_evseController->setChargeSpeed(i_pwm);
        }
        
        // Publish confirmation
        publishSpeed(i_pwm);
    }
}
