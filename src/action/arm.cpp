#include <action/arm.hpp>
#include <pico/stdlib.h>

Arm::Arm(DynamixelXL430 *deploy, DynamixelXL430 *head, float deployAngle, float foldedAngle) {
	this->xlDeploy = deploy;
	this->xlHead = head;
	this->deployAngle = deployAngle;
	this->foldedAngle = foldedAngle;
	this->setEnable(false);
}

Arm::~Arm() {
	this->setEnable(false);
}

void Arm::setEnable(bool enable) {
	this->enabled = enable;
	if (enable) {
		this->xlDeploy->setOperatingMode(XL430OperatingMode::position);
		this->xlHead->setOperatingMode(XL430OperatingMode::extendedPosition);

		this->xlDeploy->setTorque(true);
		this->xlDeploy->setLED(true);
		this->xlHead->setTorque(true);
		this->xlHead->setLED(true);
	} else {
		this->xlDeploy->setTorque(false);
		this->xlDeploy->setLED(false);
		this->xlHead->setTorque(false);
		this->xlHead->setLED(false);
		this->deployed = false;
	}
}

void Arm::deployMoveWait(float angle) {
	if (!this->enabled)
		return;
	this->xlDeploy->setPosition(angle);
	// Doesn't change moving straight away, it needs some time
	busy_wait_us(DYNAMIXEL_MOTION_WAIT_US);
	while (this->xlDeploy->isMoving())
		busy_wait_us(5000);
}

void Arm::deploy() {
	if (!this->enabled || this->deployed)
		return;
	this->deployMoveWait(this->deployAngle);
	this->deployed = true;
}

void Arm::fold() { // We can always fold
	if (!this->enabled)
		return;
	this->deployMoveWait(this->foldedAngle);
	this->deployed = false;
}

bool Arm::isEnabled() {
	return this->enabled;
}

bool Arm::isDeployed() {
	return this->deployed;
}

void Arm::turn(float angle) {
	if (!this->enabled || !this->deployed)
		return;
	this->xlHead->setPosition(this->xlHead->getPosition()+angle);
	// Doesn't change moving straight away, it needs some time
	busy_wait_us(DYNAMIXEL_MOTION_WAIT_US);
	while (this->xlHead->isMoving())
		busy_wait_us(5000);
}

float Arm::getArmAngle() {
	return this->xlDeploy->getPosition();
}

float Arm::getTurnAngle() {
	return this->xlHead->getPosition();
}