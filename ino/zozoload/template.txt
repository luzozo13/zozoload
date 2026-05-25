#include <WiFi.h>
#include <PubSubClient.h>
//#include <WiFiManager.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ESPAsyncWebServer.h>

// Replace the next variables with your SSID/Password combination
const char* ssid = "xxxxx";
const char* password = "xxxxx";

#define MQTT_SERVER "192.168.1.xx"

#define STATE_INIT 0
#define STATE_A 1
#define STATE_B 2
#define STATE_C 3
#define STATE_FAULT -1

#define TH_AB 3770
#define TH_BC 3135
#define TH_CD 2400

#define CP_AMP_8 221
#define CP_AMP_16 187
#define CP_AMP_BOOST 170
#define CP_12P 0
#define CP_12N 255

AsyncWebServer server(80);
WiFiClient espClient;
PubSubClient client(espClient);

// Pins assignment
int RX2 = 16;
int TX2 = 17;
int CP_CTRL = 18;
int FLT_CTRL = 19;
int REL_CTRL = 21;
int MT_CTRL_CLOSE = 22;
int MT_CTRL_OPEN = 23;
int L_R = 25; // Red Led
int L_G = 26; // Green Led
int L_B = 27; // Blue Led
int LOCK_ON = 32; 
int LOCK_OFF = 33;
int PILOT_READ = 34;
int B_R = 35; // Button Red
int B_G = 36; // Button Green
int B_B = 39; // Button Blue

int CH_CP_CTRL = 0; // PWM channel control signal

bool btn_start_load=true ;
bool btn_stop_load=false ;
bool b_boost=false;
String state_str;
char state[50];
bool b_OTA_flag;
bool b_publish = true;

uint16_t time_elapsed;
int CPP_value;
int CPP_max;
int CPP_min;
hw_timer_t * timer = NULL;
bool b_fault = false;
int i_nb_pos=0;
int i_state_current = STATE_INIT;
int i_state_meas = STATE_INIT;
int i_state_previous = STATE_INIT;

int i_dummy_cnt = 0;
char c_meas;
char c_current;
bool b_diff = false;


void IRAM_ATTR pub_msg(String str_msg)
{
  state_str = str_msg;
  state_str.toCharArray(state, state_str.length()+1);
  client.publish("esp32/new_state", state);
}

bool isFault(){
  return isRelayFault()| isDiodeFault();
}

bool isRelayFault(){
  // TODO
  return false;
}

bool isDiodeFault(){
  // TODO
  return false;
}

void raiseFault(){
  i_state_meas = STATE_FAULT;
  setState();
}

bool isStateDiff(){

  b_diff = i_state_meas != i_state_current;
  return i_state_meas != i_state_current;
  
}

char getStrState(int i_state_dis)
{
  char c_temp;
  if (i_state_dis == STATE_A) {c_temp = 'A';}
  if (i_state_dis == STATE_B) {c_temp = 'B';}
  if (i_state_dis == STATE_C) {c_temp = 'C';}
  return c_temp;
}

void publishDiff(){
    c_meas = getStrState(i_state_meas);
    c_current = getStrState(i_state_current);
    pub_msg("Diff: meas:" +
            String(CPP_max) +
            "(" + c_meas +
            ")current:" +
//            String(i_state_current) +
            "(" + c_current +
            ")");
    b_diff = false;
}

bool isFirstStateDiff(){
  return i_state_meas != i_state_previous;
}

void startDiffTimer(){
  // enable timer
  // restart timer
  
  //DUMMY
  i_dummy_cnt = 0;
//  pub_msg("start diff timer");
  i_state_previous = i_state_meas;
}

void stopDiffTimer(){
}

bool isDiffSteady(){
  i_dummy_cnt++;
  // if measured state A and time >= 25ms => TRUE
  // if measured state (not A) and time >= 250ms => TRUE
  // else => FALSE
  
  if (i_dummy_cnt <= 3)
  {
//    pub_msg("i_dummy_cnt: " + String(i_dummy_cnt));
    return false;
  }
  else
  {
//    pub_msg("i_dummy_cnt>3: " + String(i_dummy_cnt));
    return true;
  }
}

void setState(){
  stopDiffTimer();
  if (i_state_current == STATE_FAULT) {
    setStateFault();
    return;
  }
  switch (i_state_meas) {
    case STATE_A:
      setStateA();
      break;
    case STATE_B:
      if (i_state_current == STATE_A | i_state_current == STATE_C){
        setStateB();
      }
      else{
        setStateA();
      }
      break;
    case STATE_C:
      if (i_state_current == STATE_B){
        setStateC();
      }
      else {
        setStateA();
      }
      break;
    case STATE_FAULT:
    default:
      setStateFault();
  }
}

void setStateVariables(int i_state)
{
  i_state_current = i_state;
  i_state_previous = i_state;
  i_state_meas = i_state;
}

bool isState(int i_state)
{
  return i_state_current == i_state;
}

void setStateFault(){
  client.publish("esp32/started", "setStateFault");
  i_state_current = STATE_FAULT;
  publishState();
  digitalWrite(REL_CTRL, LOW);
  digitalWrite(FLT_CTRL, HIGH);
}

void setStateA(){
  client.publish("esp32/started", "setStateA");
  setStateVariables(STATE_A);
  ledcWrite(CH_CP_CTRL, CP_12P);
  digitalWrite(REL_CTRL, LOW);
}

void setStateB(){
  //pub_msg("setStateB");
  client.publish("esp32/started", "setStateB");
  setStateVariables(STATE_B);
  ledcWrite(CH_CP_CTRL, CP_AMP_16);
  digitalWrite(REL_CTRL, LOW);
}

void setStateC(){
  setStateVariables(STATE_C);
  client.publish("esp32/started", "setStateC");
  if (b_boost)
    ledcWrite(CH_CP_CTRL, CP_AMP_BOOST);
  else
    ledcWrite(CH_CP_CTRL, CP_AMP_16);
  digitalWrite(REL_CTRL, HIGH);
}

void IRAM_ATTR onTimer(){
  Serial.println("hello timer");
  b_publish = true;
}

void publishState(){
//  c_meas = getStrState(i_state_meas);
//  c_current = getStrState(i_state_current);

  pub_msg(String(i_state_current) + "," +
          String(i_state_meas) + "," +
          String(CPP_min) + "," +
          String(CPP_max));

//  pub_msg("max:" + String(CPP_max));
//  pub_msg("min:" + String(CPP_min));
//  pub_msg("i_nb_pos:" + String(i_nb_pos));
//  pub_msg("State:" + String(i_state_current));
  //b_publish = false;
}

void setup() {
  ledcAttachPin(CP_CTRL, CH_CP_CTRL);
  ledcSetup(CH_CP_CTRL, 1000, 8);
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

  digitalWrite(REL_CTRL, LOW);
  digitalWrite(FLT_CTRL, LOW);
  digitalWrite(L_R, LOW);
  digitalWrite(L_G, LOW);
  digitalWrite(L_B, LOW);

  b_publish = true;

//  timer = timerBegin(0, 80, true);
//  timerAttachInterrupt(timer, &onTimer, true);
//  timerAlarmWrite(timer, 4000000, true);
//  timerAlarmEnable(timer);

  // Serial link setup
  Serial.begin(115200);
  // Wifi setup
  setup_wifi();

  Serial.println("Hello1");

  ArduinoOTA.setHostname("Demo OTA ESP32");
  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  
  client.setServer(MQTT_SERVER, 1883);

  server.on("/restart", HTTP_GET, [](AsyncWebServerRequest *request){ 
    request->send(200, "text/plain", "Restarting...");
    //pub_msg(String("Reset ESP"));
    
    delay(1000);
    ESP.restart();
  });
  server.on("/setflag", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", "Set flag for OTA update...");
    b_OTA_flag=true;
    time_elapsed=0;
    client.publish("esp32/started", "Set OTA flag");
    //pub_msg(String("Set OTA flag"));

  });
  server.begin();
}

void setup_wifi() {
  delay(10);
 
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32Client-EVSE";
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      client.publish("esp32/started", "Connection OK");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


int getState()
{
  CPP_max = 0;
  CPP_min = 4095;

  //45 iterations -> ca 1ms CPU execution time
  int i_nb=0;
  for(int i=0; i<=25500; i++)
  {
    CPP_value = analogRead(PILOT_READ);
    if (CPP_max < CPP_value) {CPP_max = CPP_value;}
    if (CPP_min > CPP_value) {CPP_min = CPP_value;}
    if (CPP_value >= 2000) {i_nb++;}
  }
  i_nb_pos = i_nb;

  if (CPP_max >= TH_AB){
    i_state_meas = STATE_A;
  }
  else if (CPP_max >= TH_BC){
    i_state_meas = STATE_B;    
  }
  else if (CPP_max >= TH_CD){
    i_state_meas = STATE_C;    
  }
}


void loop() {
  // put your main code here, to run repeatedly:
  if(b_OTA_flag)
  {
    //timerAlarmDisable(timer);
    time_elapsed = 0;
    uint16_t time_start = millis();
    client.publish("esp32/started", "OTA loop start");
    while(time_elapsed < 15000)
    {
      ArduinoOTA.handle();
      time_elapsed = millis() - time_start;
      delay(10);
    }
   client.publish("esp32/started", "OTA loop stop");
   b_OTA_flag = false;
//    timerAlarmEnable(timer);
  }

  if (!client.connected()) {
    reconnect();
  }
  client.loop();


  if (btn_stop_load)
  {
    setStateB();
   delay(10000);
    setStateA();
   btn_stop_load = false;
   btn_start_load = false;
   
  }
  else if (btn_start_load)
  {
    //client.publish("esp32/started", "in loop charge");
    if (isFault()){
      raiseFault();
    }
    else {
      getState();
      if (isStateDiff()) {
  //      pub_msg("(diff)max:" + String(CPP_max));
        if (isFirstStateDiff()) {
          startDiffTimer();
        }
        else {
          if (isDiffSteady())
            setState();
        }
      }
    }
  }
  
  if (b_publish)
  {
    publishState();
  }
//  if (b_diff)
//  {
//    publishDiff();
//  }
  if (digitalRead(B_B))
  {
    client.publish("esp32/started", "Start charge");
    btn_stop_load = false;
    btn_start_load = true;  
  }
//  pub_msg("max:" + String(CPP_max));
  if (digitalRead(B_R))
  {
    client.publish("esp32/started", "Stop charge");
    btn_stop_load = true;
  }
  if (digitalRead(B_G))
  {
    if (b_boost)
    {
      client.publish("esp32/started", "Reset boost");
      b_boost = false;
    }
    else
    {
      client.publish("esp32/started", "Set boost");
      b_boost = true;
    }
    if (isState(STATE_C))
      setStateC();
  }
//  Serial.println(btn_stop_load);
}
