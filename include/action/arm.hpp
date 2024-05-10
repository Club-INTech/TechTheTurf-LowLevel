#pragma once

#include <action/dynamixel_xl430.hpp>

#define DYNAMIXEL_MOTION_WAIT_US 100000

class Arm
{
public:
	Arm(DynamixelXL430 *deploy, DynamixelXL430 *head, float deployAngle, float deployHalfAngle, float foldedAngle);
	~Arm();

	void setEnable(bool enable);

	void deploy();
	void halfDeploy();
	void fold();
	bool isDeployed();
	bool isEnabled();

	void turn(float angle);

	float getArmAngle();
	float getTurnAngle();

private:

	// Move the deploy arm to the angle and wait until it's there
	void deployMoveWait(float angle);

	bool enabled;
	bool deployed;
	bool halfDeployed;

	DynamixelXL430 *xlDeploy;
	DynamixelXL430 *xlHead;

	float deployAngle;
	float halfDeployedAngle;
	float foldedAngle;

};