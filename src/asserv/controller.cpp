#include <cmath>
#include <asserv/controller.hpp>

Controller::Controller(Odometry *odo, SpeedProfile *spDst, SpeedProfile *spAngle, float dstTolerance, float angleTolerance, float amaxDstEstop, float amaxAngleEstop) {
	this->odo = odo;
	this->spDst = spDst;
	this->spAngle = spAngle;
	this->dstTolerance = dstTolerance;
	this->angleTolerance = angleTolerance;
	this->amaxDstEstop = amaxDstEstop;
	this->amaxAngleEstop = amaxAngleEstop;
	this->estopped = false;
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

bool Controller::isReady() {
	return this->state == ControllerState::reachedTarget;
}

bool Controller::isEstopped() {
	return this->estopped;
}

ControllerState Controller::getState() {
	return this->state;
}

Target Controller::getDeltaTarget() {
	Target delta;
	delta.set(this->target.dst-this->oldTarget.dst, this->target.theta-this->oldTarget.theta);
	return delta;
}

void Controller::estop() {
	this->spDst->stop(this->amaxDstEstop);
	this->spAngle->stop(this->amaxAngleEstop);
	this->estopped = true;
}

void Controller::work(float dt) {
	if (this->state == ControllerState::reachedTarget)
		return;

	if (this->state == ControllerState::reachingTheta) {
		this->spAngle->process(dt);

		if (this->spAngle->isStopping()) {
			if (!this->spAngle->isDone())
				return;

			// In case of emergency stop, change the current target to the new target
			this->target.theta = this->oldTarget.theta + this->spAngle->getPosition();
		} else {
			if (!this->spAngle->isDone() || abs(this->target.theta-this->odo->theta) > this->angleTolerance)
				return;
		}
		
		this->state = ControllerState::reachingDst;
	}

	if (this->state == ControllerState::reachingDst) {
		this->spDst->process(dt);

		if (this->spDst->isStopping()) {
			if (!this->spDst->isDone())
				return;

			// In case of emergency stop, change the current target to the new target
			this->target.dst = this->oldTarget.dst + this->spDst->getPosition();
		} else {
			if (!this->spDst->isDone() || abs(this->target.dst-this->odo->dst) > this->dstTolerance)
				return;
		}

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
	this->estopped = false;
	this->target.set(dst, theta);
	this->spDst->initMove(dst - this->oldTarget.dst);
	this->spAngle->initMove(theta - this->oldTarget.theta);
	this->state = ControllerState::reachingTheta;
}

void Controller::setRawTarget(float dst, float theta) {
	this->estopped = false;
	this->target.set(dst, theta);
}

void Controller::reset(float dst, float theta) {
	this->target.set(dst, theta);
	this->oldTarget.set(dst, theta);
	this->state = ControllerState::reachedTarget;
	this->estopped = false;
	this->spDst->reset();
	this->spAngle->reset();
}