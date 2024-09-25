#include <algorithm>

#include <asserv/pid.hpp>

// Base implem: https://gist.github.com/bradley219/5373998
// Improvments: https://www.pm-robotix.eu/2022/01/19/ameliorer-vos-regulateurs-pid/
// More improvments: Eq 3 from https://www.controleng.com/articles/pid-correction-based-control-system-implementation/

PID::PID(float Kp, float Ki, float Kd, float ramp, float lpf, float min, float max) {
	setPID(Kp,Ki,Kd);
	setClamp(min,max);
	setLpf(lpf);
	setRamp(ramp);
	setPassthrough(false);
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

void PID::setPassthrough(bool enabled) {
	this->passthrough = enabled;
}

float PID::clampVal(float val) {
	if (this->min != 0)
		val = std::max(this->min, val);
	if (this->max != 0)
		val = std::min(this->max, val);
	return val;
}

float PID::calculate(float desired, float current, float dt) {
	if (this->passthrough) {
		float output = clampVal(desired)*this->Kp;
		this->telem.add(PIDTelemData(desired, current, output), dt);
		return output;
	}

	// Calculate error
	float error = desired - current;

	// Proportional term
	float Pout = this->Kp * error;

	// Integral term
	// Accumulate with Ki to avoid bumps when changing Ki in real time
	// Use a weighed average of the current and last error to avoid noise
	this->integral += this->Ki * (((error + this->lastError)/2.0f) * dt);
	// Clamp to avoid reset windup
	this->integral = clampVal(this->integral);
	float Iout = this->integral;

	// Derivative term
	// Only on the input to avoid derivative kick
	// Apply small FIR filter to the input to avoir noise
	float derivative = -(((current - this->lastInputs[2] + 3.0f*(this->lastInputs[0] - this->lastInputs[1]))/6.0f) / dt);
	//float derivative = -((current - this->lastInputs[0]) / dt);
	float Dout = this->Kd * derivative;

	// Calculate total output
	float output = Pout + Iout + Dout;

	// Restrict to max/min
	output = clampVal(output);

	// Save old input & error for derivative & integral
	this->lastError = error;
	for (int i=2;i>0;i--)
		this->lastInputs[i] = this->lastInputs[i-1];
	this->lastInputs[0] = current;

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
	for (int i=0;i<3;i++)
		this->lastInputs[i] = 0;
	this->lastError = 0;
	this->lastOutput = 0;
	this->lastOutputRamp = 0;
}