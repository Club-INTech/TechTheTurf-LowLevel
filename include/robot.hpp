#pragma once

#include <hardware/uart.h>
#include <hardware/i2c.h>

#ifdef ROBOT_PAMI
	// Control Loop Config
	#define POSITION_DOWNSAMPLING 4
	// rads/s
	#define MAX_VELOCITY 10.0f
	// rads/s^2
	#define MAX_ACCEL 1000.0f
	// PIDs
	#ifdef PAMINABLE
		// Speed PID
		#define SPEED_PID_KP 0.001f
		#define SPEED_PID_KI 0.0f
		#define SPEED_PID_KD 0.00005f

		// Dst PID
		#define DST_PID_KP 20.0f
		#define DST_PID_KI 1.0f
		#define DST_PID_KD 0.5f

		// Angle PID
		#define ANGLE_PID_KP 4000.0f
		#define ANGLE_PID_KI 70.0f
		#define ANGLE_PID_KD 80.0f
	#else
		// Speed PID
		#define SPEED_PID_KP 0.001f
		#define SPEED_PID_KI 0.0f
		#define SPEED_PID_KD 0.00005f

		// Dst PID
		#define DST_PID_KP 20.0f
		#define DST_PID_KI 1.0f
		#define DST_PID_KD 0.5f

		// Angle PID
		#define ANGLE_PID_KP 4000.0f
		#define ANGLE_PID_KI 70.0f
		#define ANGLE_PID_KD 80.0f
	#endif

	// I2C
	#define I2C_INSTANCE i2c0
	#define I2C_ADDR 0x69

	// Pins
	#ifdef PAMI_CARTE_2A
		// Temporary pins
		#define I2C_SDA 12
		#define I2C_SCL 13

		#define LEFT_MOTOR_FW_PIN 20
		#define LEFT_MOTOR_RW_PIN 21
		#define RIGHT_MOTOR_FW_PIN 27
		#define RIGHT_MOTOR_RW_PIN 26

		#define LEFT_INCREMENTAL_A_PIN 17
		#define LEFT_INCREMENTAL_B_PIN 16
		#define RIGHT_INCREMENTAL_A_PIN 19
		#define RIGHT_INCREMENTAL_B_PIN 18
	#else
		// Actual card pins
		#define I2C_SDA 0
		#define I2C_SCL 1

		// On the PCB, Left & Right incremental & encoder are inversed
		// from original pin mapping, so we inverse it here...
		#define LEFT_MOTOR_FW_PIN 4
		#define LEFT_MOTOR_RW_PIN 5
		#define RIGHT_MOTOR_FW_PIN 2
		#define RIGHT_MOTOR_RW_PIN 3

		#define LEFT_INCREMENTAL_A_PIN 8
		#define LEFT_INCREMENTAL_B_PIN 9
		#define RIGHT_INCREMENTAL_A_PIN 6
		#define RIGHT_INCREMENTAL_B_PIN 7
	#endif

	// Mech constants
	#ifdef PAMINABLE
		// Paminable
		#define WHEEL_RADIUS (34.0f/2.0f)
		#define ENCODER_DIST 89.0f

		#define ENCODER_LEFT_REVERSE false
		#define ENCODER_RIGHT_REVERSE true

		#define DRIVER_LEFT_REVERSE false
		#define DRIVER_RIGHT_REVERSE false
	#else
		// Pamisérable
		#define WHEEL_RADIUS (34.0f/2.0f)
		#define ENCODER_DIST 89.0f

		#define ENCODER_LEFT_REVERSE false
		#define ENCODER_RIGHT_REVERSE true

		#define DRIVER_LEFT_REVERSE false
		#define DRIVER_RIGHT_REVERSE true
	#endif
#endif

// Main Robot
#ifdef ROBOT_MAIN
	// Control Loop Config
	#define POSITION_DOWNSAMPLING 1
	// rads/s
	#define MAX_VELOCITY 100.0f
	// rads/s^2
	#define MAX_ACCEL 1000.0f
	// PIDs
	// Speed PID
	#define SPEED_PID_KP 1.0f
	#define SPEED_PID_KI 0.0f
	#define SPEED_PID_KD 0.0f

	// Dst PID
	#define DST_PID_KP 1.0f
	#define DST_PID_KI 0.0f
	#define DST_PID_KD 0.0f

	// Angle PID
	#define ANGLE_PID_KP 1.0f
	#define ANGLE_PID_KI 0.0f
	#define ANGLE_PID_KD 0.0f

	// I2C
	#define I2C_INSTANCE i2c0
	#define I2C_ADDR 0x69

	// UART BG
	#define UART_INSTANCE uart0

	// BG Motors
	#define BG_LEFT_ID 0
	#define BG_RIGHT_ID 1

	// Pins
	// Not really there but for testing
	#define I2C_SDA 0
	#define I2C_SCL 1

	#define UART_TX 12
	#define UART_RX 13


	#define LEFT_INCREMENTAL_A_PIN 8
	#define LEFT_INCREMENTAL_B_PIN 9
	#define RIGHT_INCREMENTAL_A_PIN 6
	#define RIGHT_INCREMENTAL_B_PIN 7

	// Mech Constants

	#define WHEEL_RADIUS (51.0f/2.0f)
	#define ENCODER_DIST 107.5f

	#define ENCODER_LEFT_REVERSE false
	#define ENCODER_RIGHT_REVERSE true

	#define DRIVER_LEFT_REVERSE true
	#define DRIVER_RIGHT_REVERSE true
#endif



// Testing Robots

// Didier
#ifdef ROBOT_DIDIER
	// Control Loop Config
	#define POSITION_DOWNSAMPLING 4
	// rads/s
	#define MAX_VELOCITY 10.0f
	// rads/s^2
	#define MAX_ACCEL 1000.0f
	// PIDs
	// Speed PID
	#define SPEED_PID_KP 0.001f
	#define SPEED_PID_KI 0.0f
	#define SPEED_PID_KD 0.00005f

	// Dst PID
	#define DST_PID_KP 20.0f
	#define DST_PID_KI 1.0f
	#define DST_PID_KD 0.5f

	// Angle PID
	#define ANGLE_PID_KP 4000.0f
	#define ANGLE_PID_KI 70.0f
	#define ANGLE_PID_KD 80.0f

	// I2C
	#define I2C_INSTANCE i2c0
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