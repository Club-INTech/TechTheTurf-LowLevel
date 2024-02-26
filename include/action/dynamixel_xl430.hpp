#pragma once

#include <action/dynamixel_motor.hpp>

#define XL430_MODEL_NB 1060

#define XL430_MAX_VEL 1023
#define XL430_RPM_PER_TICK 0.229

#define XL430_MAX_POS 4095
#define XL430_DEG_PER_TICK (360.0/XL430_MAX_POS)

#define XL430_MAX_PWM 885
#define XL430_PERCENT_PER_TICK (100.0/XL430_MAX_PWM)

enum XL430OperatingMode { 
	velocity = 1,
	position = 3,
	extendedPosition = 4,
	pwm = 16
};

class DynamixelXL430 : public DynamixelMotor
{
public:
	DynamixelXL430(uint8_t id);

	int setTorque(bool torque);
	int setOperatingMode(XL430OperatingMode mode);


	// Set targets to raw respective ticks
	// 0 - XL430_MAX_POS
	int setPositionRaw(uint32_t pos);
	// -XL430_MAX_PWM - XL430_MAX_PWM
	int setPwmRaw(int16_t pwm);
	// -XL430_MAX_VEL - XL430_MAX_VEL
	int setVelocityRaw(int32_t vel);

	// Set targets to values with units
	// 0 deg - 360 deg
	int setPosition(float posDeg);
	// 0 % - 100 %
	int setPwm(float pwmPercent);
	// 0 rpm - 234 rpm
	int setVelocity(float velRpm);

	// Set targets relative to min & max set in motor
	// [0, 1]
	int setPositionRel(float pos);
	// [-1, 1]
	int setPwmRel(float pwm);
	// [-1, 1]
	int setVelocityRel(float vel);

	int getShutdownStatus();

	// Reboots motor & gets min/max values
	int initMotor();
private:
	
	uint16_t pwmMax;
	uint32_t velMax;
	uint32_t posMin, posMax;
};