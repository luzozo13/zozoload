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
  client.setCallback(mqttCallback);

  ArduinoOTA.begin();
  Serial.println("OTA prêt");

  reconnect();
  client.publish("zozo-charge/OTA", "OTA prêt");

  digitalWrite(REL_CTRL, LOW);
  digitalWrite(FLT_CTRL, LOW);
  digitalWrite(L_R, HIGH);
  digitalWrite(L_G, HIGH);
  digitalWrite(L_B, HIGH);
  
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
      // Subscribe to topics after successful connection
      client.subscribe(MQTT_SET_PWM);
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
  unsigned long ul_start_time = millis();
  while (millis() - ul_start_time < 25) { // Sample for 25ms
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

void update_charge_speed() {
    int i_new_speed = i_charge_speed;

    // ######## DEBUG ##########
    char msg[16];
    // ######## END DEBUG ##########

    if (digitalRead(B_R)) {
      i_new_speed = CP_AMP_8;
      // ######## DEBUG ##########
      snprintf(msg, sizeof(msg), "B_8A");
      // ######## END DEBUG ##########
    } else if (digitalRead(B_G)) {
      i_new_speed = CP_AMP_16;
      // ######## DEBUG ##########
      snprintf(msg, sizeof(msg), "B_16A");
      // ######## END DEBUG ##########
    } else if (digitalRead(B_B)) {
      i_new_speed = CP_AMP_BOOST;
      // ######## DEBUG ##########
      snprintf(msg, sizeof(msg), "B_BOOST");
      // ######## END DEBUG ##########
    }
    if (i_new_speed != i_charge_speed) {
      // ######## DEBUG ##########
      client.publish(MQTT_SPEED, msg);
      // ######## END DEBUG ##########
      i_charge_speed = i_new_speed;
      if (i_state_current == STATE_C) {
        setStateC(); // Update state if currently charging
      }
    }
}

void loop() {

  // // ########## DEBUG ##########
  // loop_start_time = millis();
  // // ########## END DEBUG ##########


  ArduinoOTA.handle();

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // // ########## DEBUG ##########
  // getstate_start_time = millis();
  // // ########## END DEBUG ##########

  getState();

  // // ########## DEBUG ##########
  // getstate_end_time = millis();
  // getstate_duration = getstate_end_time - getstate_start_time;
  // // ########## END DEBUG ##########

  if (isStateDiff()) {
    if (isFirstStateDiff()) {
      startDiffTimer();
    }
    else {
      if (isDiffSteady()) {
        // Store previous state as a character
        char prev_state = 'U';
        if (i_state_current == STATE_A) prev_state = 'A';
        else if (i_state_current == STATE_B) prev_state = 'B';
        else if (i_state_current == STATE_C) prev_state = 'C';
        else prev_state = 'U';

        setState();

        // Store new state as a character
        char new_state = 'U';
        if (i_state_current == STATE_A) new_state = 'A';
        else if (i_state_current == STATE_B) new_state = 'B';
        else if (i_state_current == STATE_C) new_state = 'C';
        else new_state = 'U';

        // Publish the transition (e.g. "B->C")
        char transition_msg[8];
        snprintf(transition_msg, sizeof(transition_msg), "%c->%c", prev_state, new_state);
        client.publish(MQTT_CHANGE, transition_msg);
      }
    }
  }
  update_charge_speed();

  // ########### DEBUG ##########
  // Publish state to MQTT only every state_publish_interval ms
  if (millis() - last_state_publish > state_publish_interval) {
  // ########### END DEBUG ##########

    publishState();
  
  // ############ DEBUG ##########  
    last_state_publish = millis();
  }
  // ############ END DEBUG ##########

  // // ########## DEBUG ##########
  // loop_end_time = millis();
  // loop_duration = loop_end_time - loop_start_time;

  // // Publish timings to MQTT
  // char timing_msg[64];
  // snprintf(timing_msg, sizeof(timing_msg), "loop:%lums getState:%lums", loop_duration, getstate_duration);
  // client.publish("zozo-charge/timing", timing_msg);
  // // ########## END DEBUG ##########
}

// Vérifie si l'état mesuré est différent de l'état courant
bool isStateDiff() {
  return i_state_meas != i_state_current;
}

// Vérifie si c'est le premier changement d'état
bool isFirstStateDiff() {
  return i_state_meas != i_state_previous;
}

// Démarre le "timer" de stabilité de l'état
void startDiffTimer() {
  i_state_previous = i_state_meas;
  i_debounce_cnt = 0;
}

// Vérifie si l'état mesuré est stable
bool isDiffSteady() {
  if (i_state_meas == i_state_previous) {
    i_debounce_cnt++;
    if (i_debounce_cnt >= STATE_CHANGE_DEBOUNCE) {
      i_debounce_cnt = 0;
      return true;
    }
  } else {
    i_debounce_cnt = 0;
  }
  return false;
}

void setStateFault(){
  i_state_current = STATE_FAULT;
  digitalWrite(REL_CTRL, LOW);
  digitalWrite(FLT_CTRL, HIGH);
}

void setStateA(){
  i_state_current = STATE_A;
  ledcWrite(CH_CP_CTRL, CP_12P);
  digitalWrite(REL_CTRL, LOW);
}

void setStateB(){
  i_state_current = STATE_B;
  ledcWrite(CH_CP_CTRL, i_charge_speed);
  digitalWrite(REL_CTRL, LOW);
}

void setStateC(){
  i_state_current = STATE_C;
  ledcWrite(CH_CP_CTRL, i_charge_speed);
  digitalWrite(REL_CTRL, HIGH);
}

void setState() {
  switch (i_state_meas) {
    case STATE_A:
      setStateA();
      break;
    case STATE_B:
      // Only allow A->B, never C->B
      if (i_state_current == STATE_A) {
        setStateB();
      } else {
        setStateA();
      }
      break;
    case STATE_C:
      // Only allow B->C
      if (i_state_current == STATE_B) {
        setStateC();
      } else {
        setStateA();
      }
      break;
    case STATE_FAULT:
    default:
      setStateFault();
      break;
  }
}

void publishState() {
  // Convert state to character
  const char* state_char = "";
  if (i_state_current == STATE_A) state_char = "A";
  else if (i_state_current == STATE_B) state_char = "B";
  else if (i_state_current == STATE_C) state_char = "C";
  else state_char = "Unknown";

  // Compose JSON message
  char msg[64];
  snprintf(msg, sizeof(msg),
    "{\"state\":\"%s\",\"CPP_max\":%d,\"CPP_min\":%d}",
    state_char, CPP_max, CPP_min);

  client.publish(MQTT_STATE, msg);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0'; // Null-terminate first
  
  if (strcmp(topic, MQTT_SET_PWM) == 0) {
    int pwm = atoi((char*)payload); // Convert payload to int
    if (pwm < 0) pwm = 0;
    if (pwm > 255) pwm = 255;
    i_charge_speed = pwm;
    
    // Publish the new PWM value to MQTT for confirmation
    char pwm_msg[8];
    snprintf(pwm_msg, sizeof(pwm_msg), "%d", i_charge_speed);
    client.publish(MQTT_SPEED, pwm_msg);
    
    if (i_state_current == STATE_C) {
      setStateC();
    }
  }
}