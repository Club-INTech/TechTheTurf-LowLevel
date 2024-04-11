/** 
 * B-G431B-ESC1 position motion control example with encoder
 *
 */
#include <SimpleFOC.h>
#include "comm_bg.hpp"

// ----------------- Start of Config ---------------

// Build settings

// Define to get aligment values on serial port and don't do anything
//#define ALIGN_SENSOR
#define USE_PRECALIB
//#define ENABLE_DEBUG

// Motor definitions

#define VOLTAGE_SUPPLY 24
#define MOTOR_PAIRS 10

// Comm serial definitions

#define COMM_SERIAL Serial2
#define COMM_BAUD 1000000

// BG ID for asserv

#ifdef RIGHT_DRIVER
#define COMM_UID 0x1
#elif LEFT_DRIVER
#define COMM_UID 0x0
#else // No side
// Unknown driver ID
#define COMM_UID 0xFF
#endif

// Sensor aligment definitions

// Got 3.14159274 for the elec offset on both, round it up to pi
#ifdef RIGHT_DRIVER
#define SENSOR_ZERO_ELEC_OFFSET M_PI
#define SENSOR_DIRECTION Direction::CW
#elif LEFT_DRIVER
#define SENSOR_ZERO_ELEC_OFFSET M_PI
#define SENSOR_DIRECTION Direction::CW
#else // No side
// Not calibrated
#undef USE_PRECALIB
#endif

// ----------------- End of Config -----------------

// We can't calibrate and use the precalibration..
#ifdef ALIGN_SENSOR
#undef USE_PRECALIB
#endif

// Motor instance
BLDCMotor motor = BLDCMotor(MOTOR_PAIRS/*, 0.65f, 97.0f*/);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f/7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

// encoder instance
HallSensor encoder = HallSensor(A_HALL1, A_HALL2, A_HALL3, MOTOR_PAIRS);

// Comm instance
CommBG comm = CommBG(COMM_UID, COMM_SERIAL, A_USART2_RX, A_USART2_TX, motor);

// interrupt routine initialization
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}
void doC(){encoder.handleC();}

void setup() {
	// Start serial early to handle debug if enabled
	COMM_SERIAL.begin(COMM_BAUD);

#if !defined(ALIGN_SENSOR) && !defined(ENABLE_DEBUG)
	comm.begin();
#endif

#ifdef ENABLE_DEBUG
	SimpleFOCDebug::enable(&COMM_SERIAL);
#endif

	// initialize encoder sensor hardware
	encoder.init();
	// enable hall sensor hardware interrupts
	encoder.enableInterrupts(doA, doB, doC);

	// link the motor to the sensor
	motor.linkSensor(&encoder);
	
	// driver config
	// power supply voltage [V]
	driver.voltage_power_supply = VOLTAGE_SUPPLY;
	driver.init();
	// link the motor and the driver
	motor.linkDriver(&driver);
	// link current sense and the driver
	currentSense.linkDriver(&driver);

	// current sensing
	currentSense.init();
	// no need for aligning
	currentSense.skip_align = true;
	motor.linkCurrentSense(&currentSense);

	// aligning voltage [V]
	motor.voltage_sensor_align = 3;
	// index search velocity [rad/s]
	motor.velocity_index_search = 3;

	// set motion control loop to be used
	motor.controller = MotionControlType::velocity;

	motor.torque_controller = TorqueControlType::foc_current;

	// contoller configuration 
	// default parameters in defaults.h

	// velocity PI controller parameters
	motor.PID_velocity.P = 0.2;
	motor.PID_velocity.I = 0.0;
	// default voltage_power_supply
	motor.voltage_limit = VOLTAGE_SUPPLY;
	// jerk control using voltage voltage ramp
	// default value is 300 volts per sec  ~ 0.3V per millisecond
	motor.PID_velocity.output_ramp = 1000;
 
	// velocity low pass filtering time constant
	motor.LPF_velocity.Tf = 0.01;

	// angle P controller
	motor.P_angle.P = 20;
	//  maximal velocity of the position control
	motor.velocity_limit = 4;

#ifdef USE_PRECALIB
	// Setup sensor values to avoid having to recalibrate every setup
	motor.zero_electric_angle = SENSOR_ZERO_ELEC_OFFSET;
	motor.sensor_direction = SENSOR_DIRECTION;
#else
	// Calibrate so change to unknown
	motor.sensor_direction = Direction::UNKNOWN;
#endif
	
	// initialize motor
	motor.init();
	// align encoder and start FOC
	motor.initFOC();

	// Start with the motors turned off
	motor.disable();

#ifdef ALIGN_SENSOR
	// Print the calibrated info to the serial port every 5s
	for (;;) {
		COMM_SERIAL.print("Sensor dir: ");
		if (motor.sensor_direction == Direction::CW)
			COMM_SERIAL.println("CW");
		else if (motor.sensor_direction == Direction::CW)
			COMM_SERIAL.println("CCW");
		else
			COMM_SERIAL.println("UNKNOWN");

		COMM_SERIAL.print("Sensor zero: ");
		COMM_SERIAL.println(motor.zero_electric_angle,8);
		COMM_SERIAL.println("--------");
		delay(5000);
	}
#endif
}

void loop() {
	// main FOC algorithm function
	motor.loopFOC();

	// Motion control function
	motor.move();

	// Handle cmds from master
	comm.work();
}
