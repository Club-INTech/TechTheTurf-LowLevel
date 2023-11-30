#include <accel_limiter.hpp>
#include <algorithm>
	
AccelLimiter::AccelLimiter(float max) {
	this->maxAccel = max;
	reset();
}

AccelLimiter::~AccelLimiter() {

}

float AccelLimiter::limit(float val, float dt) {
	bool accel = false;
	if (this->lastValue >= 0 && val >= 0) {
		if (this->lastValue <= val)
			accel = true;
	} else if (this->lastValue < 0 && val < 0) {
		if (this->lastValue > val)
			accel = true;
	} else {
		accel = true;
	}

	if (accel)
		this->lastValue += limitValue(val, dt);
	else
		this->lastValue = val;

	return this->lastValue;
}

float AccelLimiter::limitValue(float val, float dt) {
	float accel = val - this->lastValue;
	float max = dt * this->maxAccel;

	return std::clamp(accel, -max, max);
}

void AccelLimiter::reset() {
	this->lastValue = 0;
}