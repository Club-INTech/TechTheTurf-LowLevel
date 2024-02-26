#include <math.h>
#include <asserv/controller.hpp>

Controller::Controller(Odometry *odo, SpeedProfile *sp) {
	this->odo = odo;
	this->sp = sp;
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

	return target + this->sp->getPosition();
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

void Controller::work(float dt) {
	if (this->state == ControllerState::reachedTarget)
		return;

	if (this->state == ControllerState::reachingTheta) {
		if (abs(this->target.theta-this->odo->theta) > (2.0f*(M_PI/180.0f)))
			return;
		
		this->state = ControllerState::reachingDst;
	}

	if (this->state == ControllerState::reachingDst) {
		this->sp->process(dt);

		if (!this->sp->isDone() && abs(this->target.dst-this->odo->dst) > 20.0f)
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
	this->sp->initMove(dst - this->oldTarget.dst);
	this->state = ControllerState::reachingTheta;
}

void Controller::reset(float dst, float theta) {
	this->target.set(dst, theta);
	this->oldTarget.set(dst, theta);
	this->state = ControllerState::reachedTarget;
	this->sp->reset();
}