#pragma once

#include <pico/stdlib.h>

#define STEP_TIME 1000
#define STEP_SPACE 1000
#define DIR_HOLD 200
#define DIR_SETUP 200

class StepperDriver
{
public:
	StepperDriver(uint8_t step, uint8_t dir, uint8_t slp, uint32_t stepsPerRevs, bool reverse = false);
	~StepperDriver();

	void setEnable(bool en);

	int32_t toSteps(float turns);
	float toTurns(int32_t steps);

	void stepCount(int steps);
	void rotateRad(float angle);
	void rotateTurns(float turns);
private:

	bool state;

	uint32_t stepsPerRevs;

	uint8_t step, dir, slp;
	bool reverse;
};