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

float SpeedProfile::getTotalTime() {
	float time = this->ta*2;
	if (this->trap)
		time += this->tc;
	return time;
}

float SpeedProfile::getPosition() {
	return this->pos;
}

float SpeedProfile::process(float dt) {
	if (this->done)
		return this->target;

	this->time += dt;
	float velocity = 0;

	if (this->time < this->ta) {
		velocity = this->time * this->amax;
	} else if (this->trap)  {
		if (this->time < this->ta+this->tc) {
			velocity = this->vmax;
		} else if (this->time < 2*this->ta+this->tc) {
			velocity = this->vmax - (this->time-(this->ta+this->tc)) * this->amax;
		} else {
			this->done = true;
			return this->target;
		}
	} else {
		if (this->time < 2*this->ta) {
			velocity = this->vmaxTrig - (this->time-this->ta) * this->amax;
		} else {
			this->done = true;
			return this->target;
		}
	}
	
	this->pos += this->direction * velocity*dt;
	return this->pos;
}

bool SpeedProfile::isDone() {
	return this->done;
}

void SpeedProfile::reset() {
	this->done = false;
	this->pos = 0;
	this->time = 0;
	this->direction = 0;
	this->vmaxTrig = 0;
	this->ta = 0;
	this->tc = 0;
	this->target = 0;
	this->trap = false;
}