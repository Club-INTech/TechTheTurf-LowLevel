#pragma once

#ifdef ROBOT_PAMI

// I2C
#define I2C_ADDR 0x69

// Pins
#define I2C_SDA 0
#define I2C_SCL 1
//#define I2C_SDA 12
//#define I2C_SCL 13

#ifndef PAMI_CARTE_2A
// Actual card pins
#define LEFT_MOTOR_FW_PIN 2
#define LEFT_MOTOR_RW_PIN 3
#define RIGHT_MOTOR_FW_PIN 4
#define RIGHT_MOTOR_RW_PIN 5

#define LEFT_INCREMENTAL_A_PIN 6
#define LEFT_INCREMENTAL_B_PIN 7
#define RIGHT_INCREMENTAL_A_PIN 8
#define RIGHT_INCREMENTAL_B_PIN 9

#else
// Temporary pins
#define LEFT_MOTOR_FW_PIN 20
#define LEFT_MOTOR_RW_PIN 21
#define RIGHT_MOTOR_FW_PIN 27
#define RIGHT_MOTOR_RW_PIN 26

#define LEFT_INCREMENTAL_A_PIN 17
#define LEFT_INCREMENTAL_B_PIN 16
#define RIGHT_INCREMENTAL_A_PIN 19
#define RIGHT_INCREMENTAL_B_PIN 18

#endif

// Mech constants
// Paminable
#define WHEEL_RADIUS (34.0f/2.0f)
#define ENCODER_DIST 86.8f

#define ENCODER_LEFT_REVERSE false
#define ENCODER_RIGHT_REVERSE false

#define DRIVER_LEFT_REVERSE true
#define DRIVER_RIGHT_REVERSE true

#endif

#ifdef ROBOT_DIDIER

// I2C
#define I2C_ADDR 0x69

// Pins
// Not really there but for testing
#define I2C_SDA 0
#define I2C_SCL 1

#define LEFT_MOTOR_FW_PIN 14
#define LEFT_MOTOR_RW_PIN 15
#define RIGHT_MOTOR_FW_PIN 16
#define RIGHT_MOTOR_RW_PIN 17

#define LEFT_INCREMENTAL_A_PIN 4
#define LEFT_INCREMENTAL_B_PIN 5
#define RIGHT_INCREMENTAL_A_PIN 2
#define RIGHT_INCREMENTAL_B_PIN 3

// Mech Constants

#define WHEEL_RADIUS (68.0f/2.0f)
#define ENCODER_DIST (200.0f/2.0f)

#define ENCODER_LEFT_REVERSE false
#define ENCODER_RIGHT_REVERSE false

#define DRIVER_LEFT_REVERSE false
#define DRIVER_RIGHT_REVERSE false

#endif
