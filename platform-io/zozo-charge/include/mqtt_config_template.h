#pragma once

#define MQTT_SERVER "<MQTT_SERVER_ADDRESS>"
#define MQTT_PORT   1883
#define MQTT_TOPIC  "<MQTT_TOPIC>"
#define MQTT_STATE  MQTT_TOPIC "/state/current"
#define MQTT_CHANGE MQTT_TOPIC "/state/change"
#define MQTT_SPEED  MQTT_TOPIC "/charge_speed"
#define MQTT_SET_PWM MQTT_TOPIC "/set_pwm"