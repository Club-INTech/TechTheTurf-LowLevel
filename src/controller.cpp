#include <controller.hpp>
#include <math.h>

Controller::Controller(Odometry *odo) {
	this->odo = odo;
	this->reset();
}

Controller::~Controller() {
}

float Controller::getDstTarget() {
	if (this->state == ControllerState::reachingTheta)
		return this->odo->theta;
	return this->target.dst;
}

float Controller::getAngleTarget() {
	return this->target.theta;
}

/*void Controller::gotoXY(float x, float y) {
	float dx = x - odo->x;
	float dy = y - odo->y;

	float deltaTheta = atan2(dy, dx);
	float deltaDst = sqrt(dx * dx + dy * dy);

	movePolar(deltaDst, deltaTheta);
}*/

bool Controller::canQueueMove() {
	return !this->targetQueued;
}

void Controller::work() {
	if (this->state == ControllerState::reachingTheta) {
		if (abs(this->target.theta-this->odo->theta) > (10.0f*(M_PI/180.0f)))
			return;
		
		this->state = ControllerState::reachingDst;
	}

	if (this->state == ControllerState::reachingDst) {
		if (abs(this->target.dst-this->odo->dst) > 3.0f)
			return;

		this->state = ControllerState::reachedTarget;
	}

	if (this->state == ControllerState::reachedTarget && this->targetQueued) {
		this->state = ControllerState::reachingTheta;
		//this->target = this->nextTarget;
		//this->nextTarget.reset();
		this->targetQueued = false;
	}
}

void Controller::movePolar(float dst, float theta) {
	dst += this->target.dst;
	theta += this->target.theta;

	setTarget(dst, theta);
}

void Controller::setTarget(float dst, float theta) {
	this->target.set(dst, theta);
	this->targetQueued = true;
}

void Controller::reset(float dst, float theta) {
	this->target.set(dst, theta);
	this->nextTarget.reset();
	this->targetQueued = false;
	this->state = ControllerState::reachingDst;
}