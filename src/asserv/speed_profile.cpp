#include <asserv/speed_profile.hpp>

#include <cmath>

SpeedProfile::SpeedProfile(float vmax, float amax) {
	this->vmax = vmax;
	this->amax = amax;
	this->updateLimits();
	this->reset();
}

SpeedProfile::~SpeedProfile() {

}

void SpeedProfile::updateLimits() {
	this->trapDist = (this->vmax*this->vmax)/this->amax;
}

void SpeedProfile::setVmax(float vmax) {
	this->vmax = vmax;
	this->updateLimits();
}

void SpeedProfile::setAmax(float amax) {
	this->amax = amax;
	this->updateLimits();
}

float SpeedProfile::getVmax() {
	return this->vmax;
}

float SpeedProfile::getAmax() {
	return this->amax;
}

void SpeedProfile::initMove(float distance) {
	this->reset();
	this->target = distance;

	this->direction = 1.0f;
	if (distance < 0)
		this->direction = -1.0f;

	distance = std::abs(distance);

	if (distance > this->trapDist) {
		this->trap = true;
		this->ta = this->vmax/this->amax;
		this->tc = (distance/this->vmax) - this->ta;
	} else {
		this->trap = false;
		this->vmaxTrig = sqrtf(distance*this->amax);
		this->ta = this->vmaxTrig/this->amax;
	}
}

void SpeedProfile::stop(float acceleration) {
	acceleration = std::abs(acceleration);
	if (acceleration == 0)
		acceleration = this->amax;

	this->time = 0;
	this->stopVel = this->velocity;
	this->stopAmax = acceleration;
}

float SpeedProfile::getTotalTime() {
	float time = this->ta*2;
	if (this->trap)
		time += this->tc;
	return time;
}

float SpeedProfile::getPosition() {
	return this->position;
}

float SpeedProfile::getVelocity() {
	return this->velocity;
}

float SpeedProfile::process(float dt) {
	if (this->done)
		return this->target;

	this->time += dt;
	this->velocity = 0;

	// Stop
	if (this->stopAmax != 0) {
		this->velocity = std::max(this->stopVel - this->time*this->stopAmax, 0.0f);
	} else { // Normal operation
		if (this->time < this->ta) {
			this->velocity = this->time * this->amax;
		} else if (this->trap)  {
			if (this->time < this->ta+this->tc) {
				this->velocity = this->vmax;
			} else if (this->time < 2*this->ta+this->tc) {
				this->velocity = this->vmax - (this->time-(this->ta+this->tc)) * this->amax;
			} else {
				this->done = true;
				return this->target;
			}
		} else {
			if (this->time < 2*this->ta) {
				this->velocity = this->vmaxTrig - (this->time-this->ta) * this->amax;
			} else {
				this->done = true;
				return this->target;
			}
		}
	}
	
	this->position += this->direction * this->velocity*dt;

	// If we need to stop and we're not moving anymore, change target & finish.
	if (this->stopAmax != 0 && this->velocity == 0.0f) {
		this->target = this->position;
		this->done = true;
		return this->target;
	}

	// This method of integrating will bring about some errors
	// So if we overshot the position but the time is not up yet
	// Just finish up anyways.
	if (std::abs(this->position) >= std::abs(this->target)) {
		this->done = true;
		return this->target;
	}

	return this->position;
}

bool SpeedProfile::isDone() {
	return this->done;
}

void SpeedProfile::reset() {
	this->done = false;
	this->position = 0;
	this->velocity = 0;
	this->time = 0;
	this->direction = 0;
	this->vmaxTrig = 0;
	this->ta = 0;
	this->tc = 0;
	this->stopAmax = 0;
	this->stopVel = 0;
	this->target = 0;
	this->trap = false;
}