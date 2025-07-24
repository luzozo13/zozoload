#include <WiFi.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include "wifi_credentials.h"
#include "mqtt_config.h"
#include "params.h"

// MQTT broker settings
const char* mqtt_server = MQTT_SERVER;
const int mqtt_port = MQTT_PORT;
const char* mqtt_topic = MQTT_TOPIC;
WiFiClient espClient;
PubSubClient client(espClient);

void reconnect();

void setup() {
  Serial.begin(115200);

  // Pin modes
  pinMode(FLT_CTRL, OUTPUT);
  pinMode(REL_CTRL, OUTPUT);
  pinMode(MT_CTRL_CLOSE, OUTPUT);
  pinMode(MT_CTRL_OPEN, OUTPUT);
  pinMode(L_R, OUTPUT);
  pinMode(L_G, OUTPUT);
  pinMode(L_B, OUTPUT);
  pinMode(LOCK_ON, INPUT_PULLDOWN);
  pinMode(LOCK_OFF, INPUT_PULLDOWN);
  pinMode(PILOT_READ, INPUT);
  pinMode(B_R, INPUT_PULLDOWN);
  pinMode(B_G, INPUT_PULLDOWN);
  pinMode(B_B, INPUT_PULLDOWN);

  // PWM setup for CP_CTRL
  ledcSetup(CH_CP_CTRL, 1000, 8); // channel, freq=1kHz, resolution=8 bits
  ledcAttachPin(CP_CTRL, CH_CP_CTRL);

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

  digitalWrite(REL_CTRL, LOW);
  digitalWrite(FLT_CTRL, LOW);
  digitalWrite(L_R, LOW);
  digitalWrite(L_G, LOW);
  digitalWrite(L_B, LOW);
  
  digitalWrite(MT_CTRL_CLOSE, LOW);
    // Open lock (pulse MT_CTRL_OPEN for 1 second)
  digitalWrite(MT_CTRL_OPEN, HIGH);
  delay(1000);
  digitalWrite(MT_CTRL_OPEN, LOW);

  // Set CP_CTRL to steady 12V (State A)
  ledcWrite(CH_CP_CTRL, CP_12P); // CP_12P should be defined as 0 for 100% HIGH
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

// Reads the pilot signal and determines the charging state
int getState() {
    CPP_max = 0;
    CPP_min = 4095;

    int i_nb = 0;
    for (int i = 0; i <= 25500; i++) {
        CPP_value = analogRead(PILOT_READ);
        if (CPP_max < CPP_value) { CPP_max = CPP_value; }
        if (CPP_min > CPP_value) { CPP_min = CPP_value; }
        if (CPP_value >= 2000) { i_nb++; }
    }
    i_nb_pos = i_nb;

    if (CPP_max >= TH_AB) {
        i_state_meas = STATE_A;
    } else if (CPP_max >= TH_BC) {
        i_state_meas = STATE_B;
    } else if (CPP_max >= TH_CD) {
        i_state_meas = STATE_C;
    }
    return i_state_meas;
}

void loop() {
  ArduinoOTA.handle();

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  int current_state = getState();
   // Convert state to character
  const char* state_char = "";
  if (current_state == STATE_A) state_char = "A";
  else if (current_state == STATE_B) state_char = "B";
  else if (current_state == STATE_C) state_char = "C";
  else state_char = "Unknown";

  client.publish(mqtt_topic, state_char);


  // J1772 logic
  if (current_state == STATE_A) {
    // No car: steady 12V
    ledcWrite(CH_CP_CTRL, CP_12P);
    digitalWrite(REL_CTRL, LOW);
    digitalWrite(L_G, LOW);
  } else if (current_state == STATE_B) {
    // Car detected: start PWM, relay still open
    ledcWrite(CH_CP_CTRL, CP_AMP_16);
    digitalWrite(REL_CTRL, LOW);
    digitalWrite(L_G, LOW);
  } else if (current_state == STATE_C) {
    // Car requests charging: PWM and close relay
    ledcWrite(CH_CP_CTRL, CP_AMP_16);
    digitalWrite(REL_CTRL, HIGH);
    digitalWrite(L_G, HIGH);
  }

}