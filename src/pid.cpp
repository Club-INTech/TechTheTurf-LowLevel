#include <pid.hpp>
#include <algorithm>

// https://www.pm-robotix.eu/2022/01/19/ameliorer-vos-regulateurs-pid/

PID::PID(float Kp, float Ki, float Kd, float ramp, float lpf, float min, float max) {
	setPID(Kp,Ki,Kd);
	setClamp(min,max);
	setLpf(lpf);
	setRamp(ramp);
	reset();
}

PID::~PID() {
}

void PID::setPID(float Kp, float Ki, float Kd) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
}

void PID::setClamp(float min, float max) {
	this->max = max;
	this->min = min;
	this->integral = clampVal(integral);
	this->lastOutput = clampVal(this->lastOutput);
	this->lastOutputRamp = clampVal(this->lastOutputRamp);
}

void PID::setLpf(float lpf) {
	this->lpf = lpf;
}

void PID::setRamp(float ramp) {
	this->outRamp = ramp;
}

float PID::clampVal(float val) {
	if (this->min != 0)
		val = std::max(this->min, val);
	if (this->max != 0)
		val = std::min(this->max, val);
	return val;
}

// From: https://gist.github.com/bradley219/5373998
float PID::calculate(float desired, float current, float dt) {
	// Calculate error
	float error = desired - current;

	// Proportional term
	float Pout = this->Kp * error;

	// Integral term
	// Accumulate with Ki to avoid bumps when changing Ki in real time
	this->integral += this->Ki * (error * dt);
	// Clamp to avoid reset windup
	this->integral = clampVal(this->integral);
	float Iout = this->integral;

	// Derivative term
	// Only on the input to avoid derivative kick
	float derivative = -((current - this->lastInput) / dt);
	float Dout = this->Kd * derivative;

	// Calculate total output
	float output = Pout + Iout + Dout;

	// Restrict to max/min
	output = clampVal(output);

	// Save old input for derivative
	this->lastInput = current;

	// Apply derivative limiter
	if (this->outRamp > 0) {
		float outRate = (output - this->lastOutputRamp)/dt;
		if (outRate > this->outRamp)
			output = this->lastOutputRamp + this->outRamp * dt;
		else if (outRate < -this->outRamp)
			output = this->lastOutputRamp - this->outRamp * dt;
	}

	this->lastOutputRamp = output;

	// Apply LPF if needed
	if (this->lpf > 0) {
		float alpha = 1.0f/(this->lpf*dt + 1.0f);
		output = alpha*this->lastOutput + (1.0f - alpha)*output;
	}

	this->lastOutput = output;

	this->telem.add(PIDTelemData(desired, current, output), dt);

	return output;
}

void PID::reset() {
	this->integral = 0;
	this->lastInput = 0;
	this->lastOutput = 0;
	this->lastOutputRamp = 0;
}