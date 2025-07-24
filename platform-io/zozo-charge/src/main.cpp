#include <WiFi.h>
#include <ArduinoOTA.h>
#include "wifi_credentials.h"
#include "mqtt_config.h" // Add this line
#include <PubSubClient.h>

const int ledPin = 19;
int FLT_CTRL = 19;
int REL_CTRL = 21;

// MQTT broker settings
const char* mqtt_server = MQTT_SERVER;
const int mqtt_port = MQTT_PORT;
const char* mqtt_topic = MQTT_TOPIC;
WiFiClient espClient;
PubSubClient client(espClient);

void reconnect();

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connecté. IP: ");
  Serial.println(WiFi.localIP());

  client.setServer(mqtt_server, mqtt_port);

  ArduinoOTA.begin();
  Serial.println("OTA prêt");

  reconnect();
  client.publish("zozo-charge/OTA", "OTA prêt");

  digitalWrite(ledPin, LOW);
  client.publish(mqtt_topic, "LED OFF");
  delay(10000);

}

void reconnect() {
  while (!client.connected()) {
    if (client.connect("ZozoChargeClient")) {
      // Connected
    } else {
      delay(1000);
    }
  }
}

void loop() {
  ArduinoOTA.handle();

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  client.publish(mqtt_topic, "LOOP running");
  delay(10000);
}
