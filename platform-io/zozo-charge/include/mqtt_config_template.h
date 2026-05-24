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
