#include <pid.hpp>
#include <algorithm>

// https://www.pm-robotix.eu/2022/01/19/ameliorer-vos-regulateurs-pid/

PID::PID(float Kp, float Ki, float Kd, float lpf, float min, float max) {
	setPID(Kp,Ki,Kd);
	setClamp(min,max);
	setLpf(lpf);
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
}

void PID::setLpf(float lpf) {
	this->lpf = lpf;
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

	// Apply LPF if needed
	if (this->lpf != 0)
		output = output * 1/this->lpf + this->lastOutput * (1 - 1/this->lpf);

	this->lastOutput = output;

	return output;
}

void PID::setValue(float currentValue) {
	this->currentValue = currentValue;
}

void PID::accumulate(float delta) {
	this->currentValue += delta;
}

float PID::calculateAcc(float desired, float dt) {
	return calculate(desired, this->currentValue, dt);
}

void PID::reset() {
	this->integral = 0;
	this->lastInput = 0;
	this->currentValue = 0;
	this->lastOutput = 0;
}