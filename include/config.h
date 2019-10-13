#ifndef CONFIG_H_
#define CONFIG_H_

#include <Arduino.h>

#define BAUD_RATE_DEBUG 115200
#define BAUD_RATE_CONTROL 500000

#define SERVO_YAW_PWM PB0
#define SERVO_YAW_POS PA6
#define SERVO_PITCH_PWM PB1
#define SERVO_PITCH_POS PA7
#define IMU_SCL PB10
#define IMU_SDA PB11

#define SERIAL_DEBUG Serial1

#define NUM_SERVOS 2

#endif