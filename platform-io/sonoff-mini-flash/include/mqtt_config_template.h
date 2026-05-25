#pragma once

//-- MQTT Broker Configuration --//
#define MQTT_SERVER          "<MQTT_SERVER_ADDRESS>"
#define MQTT_PORT            1883

//-- MQTT Topic to publish power measurements (override per env in platformio.ini) --//
#ifndef TOPIC_PUBLISH_POWER
  #define TOPIC_PUBLISH_POWER  "/home/power"
#endif

//-- Broadcast topic (all devices subscribe, no device prefix) --//
#define MQTT_GET_ALL  "home/get/debug/all"

//-- State topic suffixes (prepend hostname at runtime) --//
#define TOPIC_STATE_PZEM    "/state/pzem"
#define TOPIC_STATE_COMM    "/state/comm"
#define TOPIC_STATE_TIME    "/state/time"
#define TOPIC_STATE_DEBUG   "/state/debug"
#define TOPIC_OTA_STATUS    "/state/ota"

//-- Command topic suffixes (prepend hostname at runtime) --//
// Payload reference:
//   set/acquisition-rate      "1"                seconds >= 1; sensor read interval
//   set/debug                  "on" | "off"       master debug switch (gates state/pzem, state/comm, state/time)
//   set/mqtt/server           "192.168.1.22"     saved to EEPROM; restart to apply
//   set/mqtt/port             "1883"             saved to EEPROM; restart to apply
//   set/restart               "1"                any non-empty payload restarts
#define TOPIC_SET_PZEM_ACQ_RATE  "/set/acquisition-rate"
#define TOPIC_SET_DEBUG          "/set/debug"
#define TOPIC_SET_MQTT_SERVER    "/set/mqtt/server"
#define TOPIC_SET_MQTT_PORT      "/set/mqtt/port"
#define TOPIC_SET_RESTART        "/set/restart"

