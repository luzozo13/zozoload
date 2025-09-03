//-- ESP32 EVSE Controller - Main Application --//
// ESP32 EVSE Controller following OpenEVSE architectural patterns
// Author: [Your Name]
// Date: September 2025

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <string.h>
#include <stdlib.h>

#include "params.h"
#include "MqttHandler.h"
#include "wifi_credentials.h"
#include "mqtt_config.h"

//-- Global Instances --//
// MQTT client and handler
WiFiClient wifiClient;
PubSubClient pubsubClient(wifiClient);
MqttHandler g_mqttHandler(pubsubClient);

// EVSE Controller with MQTT dependency injection
EVSEController g_EvseController(g_mqttHandler);

// MQTT broker settings
const char* sz_mqtt_server = MQTT_SERVER;
const int i_mqtt_port = MQTT_PORT;
const char* sz_mqtt_topic = MQTT_TOPIC;

//-- Global MQTT Callback Bridge --//
// Required because PubSubClient needs a C-style function pointer
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  g_mqttHandler.handleMessage(topic, payload, length);
}

//-- WiFi Connection Management --//

void setup() {
  Serial.begin(115200);
  
  // Initialize pins
  pinMode(PILOT_READ, INPUT);
  pinMode(REL_CTRL, OUTPUT);
  pinMode(FLT_CTRL, OUTPUT);
  pinMode(B_R, INPUT_PULLUP);
  pinMode(B_G, INPUT_PULLUP);
  pinMode(B_B, INPUT_PULLUP);

  // Initialize PWM for Control Pilot
  ledcSetup(CH_CP_CTRL, F_PWM, PWM_RES);
  ledcAttachPin(CP_CTRL, CH_CP_CTRL);

  // Initialize in safe state
  digitalWrite(REL_CTRL, LOW);
  digitalWrite(FLT_CTRL, LOW);
  ledcWrite(CH_CP_CTRL, CP_12P);

  // WiFi connection
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected to WiFi. IP address: ");
  Serial.println(WiFi.localIP());

  // MQTT setup through MqttHandler
  g_mqttHandler.setup(sz_mqtt_server, i_mqtt_port);
  g_mqttHandler.setEVSEController(g_EvseController);
  pubsubClient.setCallback(mqttCallback);

  // OTA setup
  ArduinoOTA.setHostname("zozo-charge");
  ArduinoOTA.begin();
  
  Serial.println("EVSE Controller Ready");
}

void reconnect() {
  while (!g_mqttHandler.isConnected()) {
    Serial.print("Attempting MQTT connection...");
    if (g_mqttHandler.reconnect("zozo-charge")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(pubsubClient.state());
      Serial.println(" try again in 5 seconds");
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

  // Main update - matches OpenEVSE pattern
  g_EvseController.update();
}
