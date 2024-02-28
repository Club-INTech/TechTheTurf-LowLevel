#pragma once

#include <action/dynamixel_xl430.hpp>

class Arm
{
public:
	Arm(DynamixelXL430 *deploy, DynamixelXL430 *head, float deployAngle, float foldedAngle);
	~Arm();

	void setEnable(bool enable);

	void deploy();
	void fold();
	bool isDeployed();
	bool isEnabled();

	void turn(float angle);

private:

	// Move the deploy arm to the angle and wait until it's there
	void deployMoveWait(float angle);

	bool enabled;
	bool deployed;

	DynamixelXL430 *xlDeploy;
	DynamixelXL430 *xlHead;

	float deployAngle;
	float foldedAngle;

};