#pragma once

#include <pico/stdlib.h>

class PID
{
public:
	PID(float Kp=1.0, float Ki=0.0, float Kd=0.0, float lpf=0.0, float min=0.0, float max=0.0);
	~PID();
	
	// Config & utils
	void setPID(float Kp, float Ki, float Kd);
	void setClamp(float min, float max);
	void setLpf(float lpf);
	void reset();

	// Calc using straight values
	float calculate(float desired, float current, float dt);

	// Calc using the accumulator
	void accumulate(float delta);
	void setValue(float currentValue);
	float calculateAcc(float desired, float dt);

private:

	// Helper to clamp to PID min/max if exists
	float clampVal(float val);

	// Parameters of PID
	float Kp,Ki,Kd;
	float min,max;
	float lpf;

	// Accumulator value
	float currentValue;

	// State of PID
	float integral;
	float lastInput;
	float lastOutput;
};