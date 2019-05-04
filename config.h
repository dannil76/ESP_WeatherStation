//
// Created by Max Nilsson on 2019-05-04.
//

#ifndef CONFIG_H
#define CONFIG_H

// Sampling interval for Humidity, Temperature and Air Pressure (in milliseconds)
#define HUMID_SAMP_INTERVAL 3000
#define PRESS_SAMP_INTERVAL 2000
#define TEMP_SAMP_INTERVAL  1000

#define HUMID_TOPIC "dml/sensor00001/humidity"
#define PRESS_TOPIC "dml/sensor00001/pressure"
#define TEMP_TOPIC  "dml/sensor00001/temperature"
#define CTRL_TOPIC  "dml/sensor00001/ctrl"

#define UART_BAUD_RATE 115200

// The following defines should be sett in the "private_config.h" file
// Examples:
// #define MQTT_HOST "192.168.1.101" or "mqtt-server.mydomain.org"
// #define MQTT_PORT 1883

// #define MQTT_USER "mqtt_user_name"
// #define MQTT_PASS "mqtt_password"

#include "private_config.h"

#endif //CONFIG_H
