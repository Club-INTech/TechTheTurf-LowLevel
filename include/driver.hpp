#pragma once

#include <pico/stdlib.h>

class Driver
{
public:
	// Must be on the base of a PWM slice
	Driver(uint fin_rin, uint resolution = 2048, float freq = 40e3, float dutyOffset = 0.1);
	~Driver();
	
	// Frequency of PWM
	void setFreq(float freq);
	// Set PWM resolution (max value) 16bits max
	void setResolution(uint resolution);
	// Set the offset to the PWM duty cycle to avoid the useless range of motors
	void setDutyOffset(float offset);
	// Duty cycle + for forwards, - for backwards, 0 for idle
	void setPwm(float duty);

private:

	void updateClkdiv();

	uint pins;
	uint slice;

	bool running;
	float currentDuty;
	uint resolution;
	float frequency;
	float dutyOffset;
};