#pragma once

//-- External System Topics (Solar/House) - Customize per installation --//
#define MQTT_SOLAR_PRODUCTION_WATTS "<SOLAR_PRODUCTION_WATTS_TOPIC>"
#define MQTT_HOUSE_CONSUMPTION_WATTS "<HOUSE_CONSUMPTION_WATTS_TOPIC>"

//-- MQTT Broker Configuration --//
#define MQTT_SERVER "<MQTT_SERVER_ADDRESS>"
#define MQTT_PORT   1883
#define MQTT_TOPIC  "<MQTT_TOPIC>"

//-- EVSE State Topics --//
#define MQTT_STATE_DETAILS  MQTT_TOPIC "/state/details"
#define MQTT_STATE_CHANGE   MQTT_TOPIC "/state/change"
#define MQTT_STATE_COMM     MQTT_TOPIC "/state/comm"
#define MQTT_STATE_TIME     MQTT_TOPIC "/state/time"
#define MQTT_STATE_DEBUG    MQTT_TOPIC "/state/debug"
#define MQTT_STATE_PWM      MQTT_TOPIC "/state/pwm"

//-- Telemetry Topics --//
#define MQTT_TELEMETRY      MQTT_TOPIC "/state/pzem"
#define MQTT_SPEED          MQTT_TOPIC "/state/charge_speed"

//-- Get Topics (publish anything to trigger a fresh status response) --//
#define MQTT_GET_DEBUG      MQTT_TOPIC "/get/debug"

//-- Command Topics --//
#define MQTT_SET_CHARGE_RATE        MQTT_TOPIC "/set/charge_rate"
#define MQTT_SET_CHARGE_RATE_STATUS MQTT_TOPIC "/set/charge_rate/status"
#define MQTT_SET_DELAY              MQTT_TOPIC "/set/delay"
#define MQTT_SET_DELAY_STATUS       MQTT_TOPIC "/set/delay/status"
#define MQTT_SET_DEBUG              MQTT_TOPIC "/set/debug"
#define MQTT_SET_DEBUG_STATUS       MQTT_TOPIC "/set/debug/status"
#define MQTT_SET_DEBUG_FLAGS        MQTT_TOPIC "/set/debug_flags"
#define MQTT_SET_DEBUG_FLAGS_STATUS MQTT_TOPIC "/set/debug_flags/status"

//-- Unified configuration commands (common to all PZEM devices) --//
// The broadcast topic has no device prefix; all three devices subscribe to it.
// Unified per-device topics use m_hostname as prefix (built at runtime in MqttHandler).
// Payload reference:
//   set/hostname              "new-name"         saved to NVS; restart to apply
//   set/pzem/address          "0x01"             hex 0x01-0x0F; saved + restart to apply
//   set/pzem/acquisition-rate "5"                seconds >= 5; PZEM read interval
//   set/debug/pzem/state      "on" | "off"       maps to DBG_PZEM flag
//   set/debug/pzem/rate       "30"               seconds >= 5; state/pzem publish interval
//   set/mqtt/server           "<MQTT_SERVER_ADDRESS>"  saved to NVS; restart to apply
//   set/mqtt/port             "1883"             saved to NVS; restart to apply
//   set/restart               "1"                any non-empty payload restarts
#define MQTT_GET_ALL  "home/get/debug/all"
// Per-device unified topic suffixes (MqttHandler prepends m_hostname at runtime)
#define TOPIC_UNIFIED_HOSTNAME       "/set/hostname"
#define TOPIC_UNIFIED_PZEM_ADDRESS   "/set/pzem/address"
#define TOPIC_UNIFIED_PZEM_ACQ_RATE  "/set/pzem/acquisition-rate"
#define TOPIC_UNIFIED_DBG_PZEM_STATE "/set/debug/pzem/state"
#define TOPIC_UNIFIED_DBG_PZEM_RATE  "/set/debug/pzem/rate"
#define TOPIC_UNIFIED_MQTT_SERVER    "/set/mqtt/server"
#define TOPIC_UNIFIED_MQTT_PORT      "/set/mqtt/port"
#define TOPIC_UNIFIED_RESTART        "/set/restart"
