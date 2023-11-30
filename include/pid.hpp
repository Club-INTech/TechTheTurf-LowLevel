#pragma once

#include <pico/stdlib.h>

#include <telemetry.hpp>

struct PIDTelemData
{
	float target;
	float input;
	float output;

	PIDTelemData(float target=0, float input=0, float output=0) {
		this->target = target;
		this->input = input;
		this->output = output;
	}
};

class PID
{
public:
	PID(float Kp=1.0, float Ki=0.0, float Kd=0.0, float ramp=0.0, float lpf=0.0, float min=0.0, float max=0.0);
	~PID();
	
	// Config & utils
	void setPID(float Kp, float Ki, float Kd);
	void setClamp(float min, float max);
	// LPF in Hz
	void setLpf(float lpf);
	// Output Ramp in (pid units)/s (derivative)
	void setRamp(float ramp);
	void reset();

	// Calc using straight values
	float calculate(float desired, float current, float dt);

	// Parameters of PID
	float Kp,Ki,Kd;
	float min,max;
	float lpf,outRamp;

	Telemetry<PIDTelemData> *telem;
private:

	// Helper to clamp to PID min/max if exists
	float clampVal(float val);

	// State of PID
	float integral;
	float lastInput;
	float lastOutput;
	float lastOutputRamp;
};