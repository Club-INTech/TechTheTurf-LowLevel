#include <math.h>
#include <asserv/controller.hpp>

Controller::Controller(Odometry *odo, SpeedProfile *spDst, SpeedProfile *spAngle, float dstTolerance, float angleTolerance) {
	this->odo = odo;
	this->spDst = spDst;
	this->spAngle = spAngle;
	this->dstTolerance = dstTolerance;
	this->angleTolerance = angleTolerance;
	this->reset();
}

Controller::~Controller() {
}

float Controller::getDstTarget() {
	if (this->state == ControllerState::reachedTarget)
		return this->target.dst;

	float target = this->oldTarget.dst;
	if (this->state == ControllerState::reachingTheta)
		return target;

	return target + this->spDst->getPosition();
}

float Controller::getAngleTarget() {
	if (this->state == ControllerState::reachingTheta)
		return this->oldTarget.theta + this->spAngle->getPosition();

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

ControllerState Controller::getState() {
	return this->state;
}

void Controller::work(float dt) {
	if (this->state == ControllerState::reachedTarget)
		return;

	if (this->state == ControllerState::reachingTheta) {
		this->spAngle->process(dt);

		if (!this->spAngle->isDone() && abs(this->target.theta-this->odo->theta) > this->angleTolerance)
			return;
		
		this->state = ControllerState::reachingDst;
	}

	if (this->state == ControllerState::reachingDst) {
		this->spDst->process(dt);

		if (!this->spDst->isDone() && abs(this->target.dst-this->odo->dst) > this->dstTolerance)
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
	this->spDst->initMove(dst - this->oldTarget.dst);
	this->spAngle->initMove(theta - this->oldTarget.theta);
	this->state = ControllerState::reachingTheta;
}

void Controller::setRawTarget(float dst, float theta) {
	this->target.set(dst, theta);
}

void Controller::reset(float dst, float theta) {
	this->target.set(dst, theta);
	this->oldTarget.set(dst, theta);
	this->state = ControllerState::reachedTarget;
	this->spDst->reset();
	this->spAngle->reset();
}