//-- ESP32 EVSE Controller - Main Application --//

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <string.h>
#include <stdlib.h>

#include "params.h"
#include "EVSEController.h"
#include "MqttHandler.h"
#include "wifi_credentials.h"
#include "mqtt_config.h"
#include <PZEM004Tv30.h>

//-- Global Instances --//
WiFiClient wifiClient;
PubSubClient pubsubClient(wifiClient);
MqttHandler g_mqttHandler(pubsubClient);

// EVSE Controller with MQTT dependency injection
EVSEController g_EvseController(g_mqttHandler);

PZEM004Tv30 pzem(Serial, PZEM_RX_PIN, PZEM_TX_PIN);

void initPZEM() {
  // Initialize Serial with 9600 baud rate on pins 1,3
  Serial.begin(9600, SERIAL_8N1, PZEM_RX_PIN, PZEM_TX_PIN);
  delay(2000);
  
  if (g_mqttHandler.isConnected()) {
    g_mqttHandler.publishDebugInit("PZEM: HardwareSerial started at 9600 baud on pins 1,3");

    float test_voltage = pzem.voltage();
    if (!isnan(test_voltage) && test_voltage > 0) {
      char success_msg[60];
      snprintf(success_msg, sizeof(success_msg), "PZEM: SUCCESS! Voltage=%.1fV", test_voltage);
      g_mqttHandler.publishDebugComm(success_msg);
    } else {
      g_mqttHandler.publishDebugComm("PZEM: Communication failed - check wiring");
    }
    
    uint8_t addr = pzem.readAddress(true);
    if (addr != 0) {
      char addr_msg[40];
      snprintf(addr_msg, sizeof(addr_msg), "PZEM: Address=0x%02X", addr);
      g_mqttHandler.publishDebugComm(addr_msg);
    } else {
      g_mqttHandler.publishDebugComm("PZEM: Address read failed (0x00)");
    }
  }
}

void debugPZEM() {
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 10000) {
    lastDebug = millis();
    
    if (g_mqttHandler.isConnected()) {
      float voltage = pzem.voltage();
      float current = pzem.current();
      float power = pzem.power();
      uint8_t addr = pzem.readAddress(false);
      
      char debug_msg[200];
      snprintf(debug_msg, sizeof(debug_msg), 
        "PZEM Debug - V:%.1f, I:%.3f, P:%.1f, Addr:0x%02X", 
        voltage, current, power, addr);
      g_mqttHandler.publishDebugComm(debug_msg);
    }
  }
}

// MQTT broker settings
const char* sz_mqtt_server = MQTT_SERVER;
const int i_mqtt_port = MQTT_PORT;
const char* sz_mqtt_topic = MQTT_TOPIC;

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  g_mqttHandler.handleMessage(topic, (uint8_t*)payload, length);
}

void readAndPublishPZEM();

//-- WiFi Connection Management --//

void setup() {
  g_EvseController.setupHardware();

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  g_mqttHandler.setup(sz_mqtt_server, i_mqtt_port);
  g_mqttHandler.setEVSEController(g_EvseController);
  pubsubClient.setCallback(mqttCallback);

  reconnect();
  initPZEM();

  ArduinoOTA.setHostname("zozo-charge");
  ArduinoOTA.begin();
}

void reconnect() {
  while (!g_mqttHandler.isConnected()) {
    if (g_mqttHandler.reconnect("zozo-charge")) {
      // Connected
    } else {
      delay(5000);
    }
  }
}

void loop() {
  ArduinoOTA.handle();

  if (!g_mqttHandler.isConnected()) {
    reconnect();
  }
  g_mqttHandler.loop();
  debugPZEM();
  g_EvseController.update();
  readAndPublishPZEM();
}

void readAndPublishPZEM() {
  static unsigned long last_read = 0;
  unsigned long now = millis();
  if (now - last_read < PZEM_UPDATE_INTERVAL) return;
  last_read = now;

  float voltage = pzem.voltage();
  float current = pzem.current();
  float power = pzem.power();
  float energy = pzem.energy();
  float frequency = pzem.frequency();
  float pf = pzem.pf();

  char debug_msg[128];
  snprintf(debug_msg, sizeof(debug_msg), "Raw: V=%.1f, I=%.2f, P=%.1f, E=%.3f", voltage, current, power, energy);
  if (g_mqttHandler.isConnected()) g_mqttHandler.publishDebugRead(debug_msg);

  bool valid = false;
  if (!isnan(voltage) && !isnan(current)) valid = true;

  g_EvseController.setMeasurements(current, voltage, power, energy);

  if (valid) {
    snprintf(debug_msg, sizeof(debug_msg), "Valid: V=%.1fV, I=%.2fA, P=%.1fW, E=%.3fkWh", voltage, current, power, energy);
    if (g_mqttHandler.isConnected()) g_mqttHandler.publishDebugRead(debug_msg);
    if (g_mqttHandler.isConnected()) {
      g_mqttHandler.publishVoltage(voltage);
      g_mqttHandler.publishCurrent(current);
      g_mqttHandler.publishPower(power);
      g_mqttHandler.publishEnergy(energy);
      g_mqttHandler.publishFrequency(frequency);
      g_mqttHandler.publishPowerFactor(pf);
    }
  } else {
    if (g_mqttHandler.isConnected()) g_mqttHandler.publishDebugRead("All readings are NaN - Check connections!");
  }
}
