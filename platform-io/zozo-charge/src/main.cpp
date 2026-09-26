//-- ESP32 EVSE Controller - Main Application --//

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <esp_task_wdt.h>
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

static bool s_ota_started = false;

// ArduinoOTA needs a network interface: start it once WiFi is up (setup or later)
static void startOtaIfReady() {
  if (s_ota_started || WiFi.status() != WL_CONNECTED) return;
  ArduinoOTA.setHostname(g_mqttHandler.getHostname());
  // An OTA upload runs inside ArduinoOTA.handle() for many seconds: feed the watchdog
  ArduinoOTA.onProgress([](unsigned int, unsigned int) { esp_task_wdt_reset(); });
  ArduinoOTA.begin();
  s_ota_started = true;
}

//-- WiFi Connection Management --//

void setup() {
  g_EvseController.setupHardware();   // Relay open, CP +12V before anything else

  // Task watchdog on the loop task: a hang reboots, and setupHardware() then
  // leaves the charger in its safe state.
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
  esp_task_wdt_add(NULL);

  // Wait for WiFi, but not forever: the EVSE must run even with no network.
  // WiFi keeps retrying in the background after the timeout.
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long wifi_start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifi_start < WIFI_CONNECT_TIMEOUT_MS) {
    delay(500);
    esp_task_wdt_reset();
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
  esp_task_wdt_reset();
  initPZEM();

  startOtaIfReady();
}

// Non-blocking: at most one MQTT connection attempt every MQTT_RECONNECT_INTERVAL.
// The EVSE state machine keeps running while the broker or WiFi is down.
void reconnect() {
  static unsigned long last_attempt = 0;
  static bool attempted = false;
  if (g_mqttHandler.isConnected() || WiFi.status() != WL_CONNECTED) return;
  unsigned long now = millis();
  if (attempted && now - last_attempt < MQTT_RECONNECT_INTERVAL) return;
  attempted = true;
  last_attempt = now;
  if (g_mqttHandler.reconnect()) {
    g_mqttHandler.publishComm();
  }
}

void loop() {
  esp_task_wdt_reset();

  startOtaIfReady();
  if (s_ota_started) ArduinoOTA.handle();

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
