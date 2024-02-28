#include <action/stepper_driver.hpp>

#include <stdio.h>
#include <math.h>

// TODO: convert to PIO

StepperDriver::StepperDriver(uint8_t step, uint8_t dir, uint8_t en, uint32_t stepsPerRevs, bool reverse) {
	this->step = step;
	this->dir = dir;
	this->en = en;
	this->stepsPerRevs = stepsPerRevs;
	this->reverse = reverse;

	gpio_init(step);
	gpio_init(dir);
	gpio_init(en);

	gpio_set_dir(step, true);
	gpio_set_dir(dir, true);
	gpio_set_dir(en, true);

	gpio_put(step, false);
	gpio_put(dir, false);

	this->setEnable(false);
}

StepperDriver::~StepperDriver() {
	gpio_deinit(this->step);
	gpio_deinit(this->dir);
	gpio_deinit(this->en);
}

void StepperDriver::setEnable(bool en) {
	this->state = en;
	gpio_put(this->en, !en);
}

int32_t StepperDriver::toSteps(float turns) {
	return turns * this->stepsPerRevs;
}

float StepperDriver::toTurns(int32_t steps) {
	return steps / this->stepsPerRevs;
}

void StepperDriver::stepCount(int steps) {
	if (!this->state || steps == 0)
		return;

	if (this->reverse)
		steps = -steps;

	if (steps < 0) {
		gpio_put(this->dir, false);
		steps = -steps;
	} else {
		gpio_put(this->dir, true);
	}

	busy_wait_us(DIR_SETUP);

	for (int i=0;i<steps;i++) {
		busy_wait_us(STEP_SPACE);
		gpio_put(this->step, true);
		busy_wait_us(STEP_TIME);
		gpio_put(this->step, false);
	}

	busy_wait_us(DIR_HOLD);
}

void StepperDriver::rotateRad(float angle) {
	this->rotateTurns(angle/(2.0*M_PI));
}

void StepperDriver::rotateTurns(float turns) {
	this->stepCount(this->toSteps(turns));
}