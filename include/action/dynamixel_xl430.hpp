#pragma once

#include <action/dynamixel_motor.hpp>

#define XL430_MODEL_NB 1060

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

	// [0, 1]
	int setPositionTarget(float pos);
	// [-1, 1]
	int setPwmTarget(float pwm);
	// [-1, 1]
	int setVelocityTarget(float vel);

	int getShutdownStatus();

	// Reboots motor & gets min/max values
	int initMotor();
private:
	
	uint16_t pwmMax;
	uint32_t velMax;
	uint32_t posMin, posMax;
};