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
		return this->oldTarget.dst;
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

bool Controller::isReady() {
	return this->state == ControllerState::reachedTarget;
}

void Controller::work() {
	if (this->state == ControllerState::reachedTarget)
		return;

	if (this->state == ControllerState::reachingTheta) {
		if (abs(this->target.theta-this->odo->theta) > (2.0f*(M_PI/180.0f)))
			return;
		
		this->state = ControllerState::reachingDst;
	}

	if (this->state == ControllerState::reachingDst) {
		if (abs(this->target.dst-this->odo->dst) > 20.0f)
			return;

		this->state = ControllerState::reachedTarget;
	}
}

void Controller::movePolar(float dst, float theta) {
	dst += this->target.dst;
	theta += this->target.theta;

	setTarget(dst, theta);
}

void Controller::setTarget(float dst, float theta) {
	this->oldTarget = this->target;
	this->target.set(dst, theta);
	this->state = ControllerState::reachingTheta;
}

void Controller::reset(float dst, float theta) {
	this->target.set(dst, theta);
	this->oldTarget.set(dst, theta);
	this->state = ControllerState::reachedTarget;
}