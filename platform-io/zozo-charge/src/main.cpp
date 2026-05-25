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
  Serial.begin(9600, SERIAL_8N1, PZEM_RX_PIN, PZEM_TX_PIN);
  delay(2000);

  // Program PZEM Modbus address from NVS (broadcast write)
  pzem.setAddress(g_mqttHandler.getPzemAddr());

  float test_voltage = pzem.voltage();
  bool pzem_ok = (!isnan(test_voltage) && test_voltage > 0);
  g_mqttHandler.setPzemOk(pzem_ok);
  g_mqttHandler.publishComm();
}

// MQTT broker settings (overridden by NVS after loadFromNVS)
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

  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
  tzset();

  // Load NVS first so hostname, server, port, acq_rate are available
  g_mqttHandler.loadFromNVS();

  g_mqttHandler.setup();   // uses NVS-stored server + port
  g_mqttHandler.setEVSEController(g_EvseController);
  pubsubClient.setCallback(mqttCallback);

  reconnect();
  initPZEM();

  ArduinoOTA.setHostname(g_mqttHandler.getHostname());
  ArduinoOTA.begin();
}

void reconnect() {
  while (!g_mqttHandler.isConnected()) {
    if (g_mqttHandler.reconnect()) {
      g_mqttHandler.publishComm();
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
  g_EvseController.update();

  // Handle home/get/debug/all: do a fresh PZEM read then publish all state
  if (g_mqttHandler.isPollRequested()) {
    g_mqttHandler.clearPollRequest();
    float voltage   = pzem.voltage();
    float current   = pzem.current();
    float power     = pzem.power();
    float energy    = pzem.energy();
    float frequency = pzem.frequency();
    float pf        = pzem.pf();
    g_EvseController.setMeasurements(current, voltage, power, energy);
    g_mqttHandler.publishTelemetry(voltage, current, power, energy, frequency, pf);
    g_mqttHandler.publishAllDebug();
  }

  readAndPublishPZEM();
  g_mqttHandler.publishTime();
}

void readAndPublishPZEM() {
  static unsigned long last_read = 0;
  unsigned long now = millis();
  if (now - last_read < (unsigned long)g_mqttHandler.getPzemAcqRate() * 1000UL) return;
  last_read = now;

  float voltage = pzem.voltage();
  float current = pzem.current();
  float power = pzem.power();
  float energy = pzem.energy();
  float frequency = pzem.frequency();
  float pf = pzem.pf();

  bool valid = !isnan(voltage) && !isnan(current);

  if (valid) {
    // valid read — telemetry published below
  } else {
    // invalid read — check connections
  }

  g_EvseController.setMeasurements(current, voltage, power, energy);
  g_mqttHandler.publishTelemetry(voltage, current, power, energy, frequency, pf);
}
