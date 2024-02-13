#pragma once

#include <pico/stdlib.h>

#include <asserv/driver_base.hpp>

class Driver : public DriverBase
{
public:
	// Must be on the base of a PWM slice
	Driver(uint fin, uint rin, bool reversed = false, uint resolution = 2048, float freq = 40e3, float dutyOffset = 0.1);
	~Driver();
	
	// Frequency of PWM
	void setFreq(float freq);
	// Set PWM resolution (max value) 16bits max
	void setResolution(uint resolution);
	// Set the offset to the PWM duty cycle to avoid the useless range of motors
	void setDutyOffset(float offset);
	// Duty cycle: + for forwards, - for backwards, 0 for idle (not braking)
	void setPwm(float duty);

	// Low level operations

	// ClkDiv settings
	// Will get overriden by calls to setFreq
	// div in [0.0f, 256.0f[
	void setClkDiv(float div);
	// divInt in [0, 255], divFrac in [0, 15]
	void setClkDiv(uint8_t divInt, uint8_t divFrac);

	// Raw Duty Cycle change, 0.0-1.0, no direction
	void setRawPwm(float duty);
private:

	uint pins;
	uint slice;
	bool reversed;

	bool running;
	float currentDuty;
	uint resolution;
	float frequency;
	float dutyOffset;
};