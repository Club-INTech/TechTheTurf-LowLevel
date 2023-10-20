#include <controller.hpp>
#include <math.h>

Controller::Controller(Odometry *odo) {
	this->odo = odo;
	this->targetTheta = 0.0f;
	this->targetDst = 0.0f;
	this->oldTargetTheta = 0.0f;
	this->oldTargetDst = 0.0f;
}

Controller::~Controller() {

}

float Controller::getDstTarget() {
	return this->targetDst;
	/*if (abs(this->odo->theta - this->targetTheta) < 0.2f) {
		this->oldTargetDst = this->targetDst;
		return this->targetDst;
	} else {
		return this->oldTargetDst;
	}*/

}

float Controller::getAngleTarget() {
	return this->targetTheta;
}

void Controller::gotoXY(float x, float y) {
	float dx = x - odo->x;
	float dy = y - odo->y;

	float deltaTheta = atan2(dy, dx);
	float deltaDst = sqrt(dx * dx + dy * dy);

	movePolar(deltaDst, deltaTheta);
}

void Controller::movePolar(float dst, float theta) {
	this->oldTargetTheta = this->targetTheta;
	this->oldTargetDst = this->targetDst;

	this->targetTheta += theta;
	this->targetDst += dst;
}

void Controller::setTarget(float dst, float theta) {
	this->oldTargetTheta = this->targetTheta;
	this->oldTargetDst = this->targetDst;

	this->targetTheta = theta;
	this->targetDst = dst;	
}

void Controller::reset(float dst, float theta) {
	this->targetDst = dst;
	this->targetTheta = theta;
	this->oldTargetDst = dst;
	this->oldTargetTheta = theta;
}