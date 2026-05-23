//-- MQTT Handler Implementation --//
// Minimalist MQTT communication handler

#include "MqttHandler.h"
#include "EVSEController.h"
#include "params.h" // For constants
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
        subscribe(MQTT_CMD);
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

void MqttHandler::publishState(const char* state, int cpp_max, int cpp_min, bool charging_enabled) {
    if (!isConnected()) return;
    
    // Compose JSON message
    char sz_msg[80];
    snprintf(sz_msg, sizeof(sz_msg),
        "{\"state\":\"%s\",\"CPP_max\":%d,\"CPP_min\":%d,\"charging\":%s}",
        state, cpp_max, cpp_min, charging_enabled ? "true" : "false");
    
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

//-- Current Measurement Publishing --//

void MqttHandler::publishCurrent(float current) {
    if (!isConnected()) return;
    
    char sz_current_msg[16];
    snprintf(sz_current_msg, sizeof(sz_current_msg), "%.2f", current);
    publish(MQTT_CURRENT, sz_current_msg);
}

void MqttHandler::publishVoltage(float voltage) {
    if (!isConnected()) return;
    
    char sz_voltage_msg[16];
    snprintf(sz_voltage_msg, sizeof(sz_voltage_msg), "%.1f", voltage);
    publish(MQTT_VOLTAGE, sz_voltage_msg);
}

void MqttHandler::publishPower(float power) {
    if (!isConnected()) return;
    
    char sz_power_msg[16];
    snprintf(sz_power_msg, sizeof(sz_power_msg), "%.1f", power);
    publish(MQTT_POWER, sz_power_msg);
}

void MqttHandler::publishEnergy(float energy) {
    if (!isConnected()) return;
    
    char sz_energy_msg[16];
    snprintf(sz_energy_msg, sizeof(sz_energy_msg), "%.3f", energy);
    publish(MQTT_ENERGY, sz_energy_msg);
}

void MqttHandler::publishFrequency(float frequency) {
    if (!isConnected()) return;
    
    char sz_frequency_msg[16];
    snprintf(sz_frequency_msg, sizeof(sz_frequency_msg), "%.1f", frequency);
    publish(MQTT_FREQUENCY, sz_frequency_msg);
}

void MqttHandler::publishPowerFactor(float pf) {
    if (!isConnected()) return;
    
    char sz_pf_msg[16];
    snprintf(sz_pf_msg, sizeof(sz_pf_msg), "%.2f", pf);
    publish(MQTT_POWER_FACTOR, sz_pf_msg);
}

void MqttHandler::publishDebug(const char* message) {
    if (!isConnected()) return;
    publish(MQTT_DEBUG, message);
}

void MqttHandler::publishDebugInit(const char* message) {
    if (!isConnected()) return;
    publish(MQTT_DEBUG_INIT, message);
}

void MqttHandler::publishDebugRead(const char* message) {
    if (!isConnected()) return;
    publish(MQTT_DEBUG_READ, message);
}

void MqttHandler::publishDebugComm(const char* message) {
    if (!isConnected()) return;
    publish(MQTT_DEBUG_COMM, message);
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

    if (strcmp(topic, MQTT_CMD) == 0) {
        if (strcmp((char*)payload, "start") == 0) {
            if (m_evseController) m_evseController->setChargingEnabled(true);
            publish(MQTT_CMD "/status", "started");
        } else if (strcmp((char*)payload, "stop") == 0) {
            if (m_evseController) m_evseController->setChargingEnabled(false);
            publish(MQTT_CMD "/status", "stopped");
        }
    }
}
