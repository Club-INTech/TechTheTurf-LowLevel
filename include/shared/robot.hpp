#pragma once

#include <hardware/uart.h>
#include <hardware/i2c.h>

#ifdef ASSERV

#ifdef ROBOT_PAMI
	// Control Loop Config
	#define POSITION_DOWNSAMPLING 4
	// rads/s
	#define MAX_VELOCITY 1000.0f
	// rads/s^2
	#define MAX_ACCEL 4000.0f
	// mm/s
	#define MAX_LIN_VELOCITY 600.0f
	// mm/s^2
	#define MAX_LIN_ACCEL 1000.0f
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
		#define ENCODER_WHEEL_RADIUS (34.0f/2.0f)
		#define ENCODER_DIST 89.0f

		#define ENCODER_LEFT_REVERSE false
		#define ENCODER_RIGHT_REVERSE true

		#define DRIVER_LEFT_REVERSE false
		#define DRIVER_RIGHT_REVERSE false
	#else
		// Pamisérable
		#define ENCODER_WHEEL_RADIUS (34.0f/2.0f)
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
	#define POSITION_DOWNSAMPLING 4
	// rads/s
	#define MAX_VELOCITY 30.0f
	// rads/s^2
	#define MAX_ACCEL 60.0f
	// mm/s
	#define MAX_LIN_VELOCITY 600.0f
	// mm/s^2
	#define MAX_LIN_ACCEL 1000.0f
	// PIDs
	// Speed PID
	#define SPEED_PID_KP 1.0f
	#define SPEED_PID_KI 0.0f
	#define SPEED_PID_KD 0.0f

	// Dst PID
	#define DST_PID_KP 1.0f
	#define DST_PID_KI 0.0f
	#define DST_PID_KD 0.1f

	// Angle PID
	#define ANGLE_PID_KP 40.0f
	#define ANGLE_PID_KI 2.0f
	#define ANGLE_PID_KD 0.0f

	// I2C
	#define I2C_INSTANCE i2c0
	#define I2C_ADDR 0x69

	// UART BG
	#define BG_UART_INSTANCE uart0

	// BG Motors
	#define BG_LEFT_ID 0
	#define BG_RIGHT_ID 1

	// Pins
	#define I2C_SDA 0
	#define I2C_SCL 1

	// BG UART Pins
	#define BG_UART_TX 12
	#define BG_UART_RX 13


	#define LEFT_INCREMENTAL_A_PIN 8
	#define LEFT_INCREMENTAL_B_PIN 9
	#define RIGHT_INCREMENTAL_A_PIN 6
	#define RIGHT_INCREMENTAL_B_PIN 7

	// Mech Constants

	#define ENCODER_WHEEL_RADIUS (53.5f/2.0f)
	#define ENCODER_DIST 114.0f

	#define ENCODER_LEFT_REVERSE false
	#define ENCODER_RIGHT_REVERSE true

	#define DRIVER_LEFT_REVERSE false
	#define DRIVER_RIGHT_REVERSE true
#endif

#else // ! ASSERV

#ifdef ROBOT_MAIN
	// I2C
	#define I2C_INSTANCE i2c0
	#define I2C_ADDR 0x68

	// UART Dynamixel
	#define DYN_UART_INSTANCE uart0
	#define DYN_BAUDRATE 57600
	#define DYN_PROTO_VER 2.0

	//#define ARM_DEPLOY_DYN_ID 1
	//#define ARM_TURN_DYN_ID 14
	#define ARM_DEPLOY_DYN_ID 16
	#define ARM_TURN_DYN_ID 15

	// Stepper for elevator
	#define ELEVATOR_STEPS_PER_ROT 200

	// Pins
	#define I2C_SDA 0
	#define I2C_SCL 1

	#define DYN_UART_TX 12
	#define DYN_UART_RX 13

	#define ELEVATOR_STEP 3
	#define ELEVATOR_DIR 2
	#define ELEVATOR_EN 4

	#define ELEVATOR_ENDSTOP 5

	#define PUMP0_PIN 6
	#define PUMP0_SOLENOID_PIN 7

	// Mech constants

	// Not real maximum distance, but what is possible physicially 
	#define ELEVATOR_MAX_PHY_DST 250.0
	// Max usable distance
	#define ELEVATOR_MAX_DST 200.0
	#define ELEVATOR_MM_PER_TURN 40.0
	#define ELEVATOR_REVERSE true

	// Solar pannel arm angles
	//#define ARM_DEPLOYED_ANGLE 270.0
	//#define ARM_FOLDED_ANGLE 354.0
	#define ARM_DEPLOYED_ANGLE 184.0
	#define ARM_FOLDED_ANGLE 146.0

#endif

#endif