#pragma once

#include <hardware/uart.h>
#include <hardware/i2c.h>
#include <cmath>

#ifdef ASSERV

// Global Asserv defines

// 500Hz
#define ASSERV_PERIOD_US 2000

// Specific per-robot defines

#ifdef ROBOT_PAMI
	// Control Loop Config
	#define POSITION_DOWNSAMPLING 4

	// Absolute limits on the motor control
	// rads/s
	#define MAX_VELOCITY 1000.0f
	// rads/s^2
	#define MAX_ACCEL 4000.0f

	#ifdef PAMINABLE
		// Trapezoidal profile for distance & angle
		// mm/s
		#define MAX_LIN_VELOCITY 300.0f
		// mm/s^2
		#define MAX_LIN_ACCEL 600.0f
		// mm/s^2
		#define MAX_LIN_ESTOP_ACCEL 2000.0f
		// rad/s
		#define MAX_TURN_VELOCITY 6.0f
		// rad/s^2
		#define MAX_TURN_ACCEL 3.0f
		// rad/s^2
		#define MAX_TURN_ESTOP_ACCEL 6.0f
	#else
		// Trapezoidal profile for distance & angle
		// mm/s
		#define MAX_LIN_VELOCITY 700.0f
		// mm/s^2
		#define MAX_LIN_ACCEL 800.0f
		// mm/s^2
		#define MAX_LIN_ESTOP_ACCEL 2000.0f
		// rad/s
		#define MAX_TURN_VELOCITY 6.0f
		// rad/s^2
		#define MAX_TURN_ACCEL 3.0f
		// rad/s^2
		#define MAX_TURN_ESTOP_ACCEL 6.0f
	#endif

	// Tolerances for the controller
	// mm
	#define TOLERANCE_DST 20.0f
	// rad
	#define TOLERANCE_ANGLE (4.0f*(M_PI/180.0f))

	// PIDs & PWM Settings
	#ifdef PAMINABLE
		// Speed PID
		#define SPEED_PID_KP 0.001f
		#define SPEED_PID_KI 0.0f
		#define SPEED_PID_KD 0.00005f

		// Dst PID
		#define DST_PID_KP 20.0f
		#define DST_PID_KI 1.0f
		#define DST_PID_KD 0.5f
		#define DST_PID_CLAMP 3000.0f

		// Angle PID
		#define ANGLE_PID_KP 4500.0f
		#define ANGLE_PID_KI 120.0f
		#define ANGLE_PID_KD 80.0f
		#define ANGLE_PID_CLAMP 100000.0f

		// PWM Driver Settings
		#define DRIVER_DUTY_OFFSET 0.05f
		// Limit Voltage on smaller motors
		#define DRIVER_DUTY_CLAMP 1.0f
	#else
		// Speed PID
		#define SPEED_PID_KP 0.001f
		#define SPEED_PID_KI 0.0f
		#define SPEED_PID_KD 0.00005f

		// Dst PID
		#define DST_PID_KP 20.0f
		#define DST_PID_KI 1.0f
		#define DST_PID_KD 0.5f
		#define DST_PID_CLAMP 3000.0f

		// Angle PID
		#define ANGLE_PID_KP 1200.0f
		#define ANGLE_PID_KI 130.0f
		#define ANGLE_PID_KD 80.0f
		#define ANGLE_PID_CLAMP 100000.0f

		// PWM Driver Settings
		#define DRIVER_DUTY_OFFSET 0.03f
		// Limit Voltage on smaller motors
		#define DRIVER_DUTY_CLAMP 0.83f
	#endif

	// I2C
	#define I2C_COMM_INSTANCE i2c0
	#define I2C_ADDR 0x69

	// Pins
	#define I2C_SDA 0
	#define I2C_SCL 1

	// On the PCB, Left & Right incremental & encoder are inversed
	// from original pin mapping, so we inverse it here...
	#ifdef PAMINABLE
		#define LEFT_MOTOR_FW_PIN 4
		#define LEFT_MOTOR_RW_PIN 5
		#define RIGHT_MOTOR_FW_PIN 2
		#define RIGHT_MOTOR_RW_PIN 3
	#else // Pamini
		#define LEFT_MOTOR_FW_PIN 2
		#define LEFT_MOTOR_RW_PIN 3
		#define RIGHT_MOTOR_FW_PIN 4
		#define RIGHT_MOTOR_RW_PIN 5
	#endif

	#define LEFT_INCREMENTAL_A_PIN 8
	#define LEFT_INCREMENTAL_B_PIN 9
	#define RIGHT_INCREMENTAL_A_PIN 6
	#define RIGHT_INCREMENTAL_B_PIN 7

	#ifndef PAMINABLE // Pami
		#define ENABLE_EFFECTS

		#define STOP_LIGHT_LEFT_PIN 10
		#define STOP_LIGHT_RIGHT_PIN 11
		#define STOP_LIGHT_CENTER_PIN 14
		#define BLINKER_LEFT_PIN 12
		#define BLINKER_RIGHT_PIN 13
		#define HEADLIGHT_LEFT_PIN 17
		#define HEADLIGHT_RIGHT_PIN 16
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
		// No name pami
		#define ENCODER_WHEEL_RADIUS (53.754f/2.0f)
		#define ENCODER_DIST 63.0f

		#define ENCODER_LEFT_REVERSE false
		#define ENCODER_RIGHT_REVERSE true

		#define DRIVER_LEFT_REVERSE false
		#define DRIVER_RIGHT_REVERSE false
	#endif
#endif

// Main Robot
#ifdef ROBOT_MAIN
	// Control Loop Config
	#define POSITION_DOWNSAMPLING 4

	// Enable to use ODrive, otherwise uses BGs
	//#define ROBOT_MAIN_ODRIVE

	// Absolute limits on the motor control
	// rads/s
	#define MAX_VELOCITY 30.0f
	// rads/s^2
	#define MAX_ACCEL 60.0f

	// Trapezoidal profile for distance & angle
	// mm/s
	#define MAX_LIN_VELOCITY 500.0f
	// mm/s^2
	#define MAX_LIN_ACCEL 1000.0f
	// mm/s^2
	#define MAX_LIN_ESTOP_ACCEL 2000.0f
	// rad/s
	#define MAX_TURN_VELOCITY 6.0f
	// rad/s^2
	#define MAX_TURN_ACCEL 3.0f
	// rad/s^2
	#define MAX_TURN_ESTOP_ACCEL 6.0f

	// Tolerances for the controller
	// mm
	#define TOLERANCE_DST 5.0f
	// rad
	#define TOLERANCE_ANGLE (1.0f*(M_PI/180.0f))

	// PIDs
	// Speed PID
	#define SPEED_PID_KP 1.0f
	#define SPEED_PID_KI 0.0f
	#define SPEED_PID_KD 0.0f

	// Dst PID
	#define DST_PID_KP 1.0f
	#define DST_PID_KI 0.0f
	#define DST_PID_KD 0.1f
	#define DST_PID_CLAMP 3000.0f

	// Angle PID
	#define ANGLE_PID_KP 80.0f
	#define ANGLE_PID_KI 5.0f
	#define ANGLE_PID_KD 10.0f
	#define ANGLE_PID_CLAMP 100000.0f

	// I2C
	#define I2C_COMM_INSTANCE i2c0
	#define I2C_ADDR 0x69

	// UART BG/ODrive
	#define UART_BG_INSTANCE uart0

	// BG Motors
	#define BG_LEFT_ID 0
	#define BG_RIGHT_ID 1

	// ODrive Motors
	#define ODRIVE_LEFT_AXIS 0
	#define ODRIVE_RIGHT_AXIS 1

	// Pins
	#define I2C_SDA 0
	#define I2C_SCL 1

	// BG/ODrive UART Pins
	#define UART_TX 16
	#define UART_RX 17

	#define LEFT_INCREMENTAL_A_PIN 8
	#define LEFT_INCREMENTAL_B_PIN 9
	#define RIGHT_INCREMENTAL_A_PIN 6
	#define RIGHT_INCREMENTAL_B_PIN 7

	// Mech Constants

	#define ENCODER_WHEEL_RADIUS (53.754f/2.0f)
	#define ENCODER_DIST 115.2f

	#define ENCODER_LEFT_REVERSE false
	#define ENCODER_RIGHT_REVERSE true

	#define DRIVER_LEFT_REVERSE false
	#define DRIVER_RIGHT_REVERSE true
#endif

#else // ! ASSERV

#ifdef ROBOT_MAIN
	// I2C
	#define I2C_COMM_INSTANCE i2c0
	#define I2C_ADDR 0x68

	// UART Dynamixel
	#define DYN_UART_INSTANCE uart0
	#define DYN_BAUDRATE 57600
	#define DYN_PROTO_VER 2.0

	#define LEFT_ARM_DEPLOY_DYN_ID 16
	#define LEFT_ARM_TURN_DYN_ID 7
	#define RIGHT_ARM_DEPLOY_DYN_ID 10
	#define RIGHT_ARM_TURN_DYN_ID 15

	// Stepper for elevator
	#define ELEVATOR_STEPS_PER_ROT 200

	// Pins
	#define I2C_SDA 0
	#define I2C_SCL 1

	#define DYN_UART_TX 16
	#define DYN_UART_RX 17

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
	#define LEFT_ARM_DEPLOYED_ANGLE 84.0
	#define LEFT_ARM_HALF_DEPLOYED_ANGLE 123.0
	#define LEFT_ARM_FOLDED_ANGLE 200.0
	#define RIGHT_ARM_DEPLOYED_ANGLE 187.0
	#define RIGHT_ARM_HALF_DEPLOYED_ANGLE 147.0
	#define RIGHT_ARM_FOLDED_ANGLE 75.0

#endif

#ifdef ROBOT_PAMI
#ifdef PAMIGAMI
	// UART Dynamixel
	#define DYN_UART_INSTANCE uart0
	#define DYN_BAUDRATE 57600
	#define DYN_PROTO_VER 2.0

	#define LEFT_WHEEL_DYN_ID 3
	#define RIGHT_WHEEL_DYN_ID 4
	#define ARM_DYN_ID 5

	// Pins
	#define DYN_UART_TX 16
	#define DYN_UART_RX 17

	#define JUMPER_PIN 2
	#define SIDE_PIN 18

	#define HCSR04_TRIG 10
	#define HCSR04_ECHO 11

	// Mech constants
	#define ARM_STANDBY_ANGLE 180.0f
	#define ARM_YELLOW_ANGLE 90.0f
	#define ARM_BLUE_ANGLE 270.0f

	#define ESTOP_DIST 100.0f

	#define WHEEL_RADIUS (54.0f/2.0f)
	#define DISTANCE_TO_RUN (110.0f+125.0f+325.0f/2.0f)
	#define ANGLES_TO_RUN (DISTANCE_TO_RUN/(2*M_PI*WHEEL_RADIUS))*360.0f
#endif
#endif

#endif