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

  float test_voltage = pzem.voltage();
  bool pzem_ok = (!isnan(test_voltage) && test_voltage > 0);
  g_mqttHandler.setPzemOk(pzem_ok);
  g_mqttHandler.publishComm();
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

  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
  tzset();

  g_mqttHandler.setup(sz_mqtt_server, i_mqtt_port);
  g_mqttHandler.setEVSEController(g_EvseController);
  pubsubClient.setCallback(mqttCallback);

  reconnect();
  g_mqttHandler.loadFromNVS();
  initPZEM();

  ArduinoOTA.setHostname("zozo-charge");
  ArduinoOTA.begin();
}

void reconnect() {
  while (!g_mqttHandler.isConnected()) {
    if (g_mqttHandler.reconnect("zozo-charge")) {
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
  readAndPublishPZEM();
  g_mqttHandler.publishTime();
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

  bool valid = !isnan(voltage) && !isnan(current);

  if (valid) {
    // valid read — telemetry published below
  } else {
    // invalid read — check connections
  }

  g_EvseController.setMeasurements(current, voltage, power, energy);
  g_mqttHandler.publishTelemetry(voltage, current, power, energy, frequency, pf);
}
