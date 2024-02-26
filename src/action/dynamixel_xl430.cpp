#include <action/dynamixel_xl430.hpp>

#include <pico/stdlib.h>

DynamixelXL430::DynamixelXL430(uint8_t id) : DynamixelMotor(id, 2.0) {
}

int DynamixelXL430::setTorque(bool torque) {
	return this->write1(64, torque ? 1 : 0);
}

int DynamixelXL430::setOperatingMode(XL430OperatingMode mode) {
	return this->write1(11, mode);
}

int DynamixelXL430::getShutdownStatus() {
	uint8_t data;
	int res = this->read1(63, &data);

	if (res != 0)
		return res;

	return data;
}


int DynamixelXL430::setPositionRaw(uint32_t pos) {
	return this->write4(116, pos);
}

int DynamixelXL430::setPwmRaw(int16_t pwm) {
	return this->write2(100, pwm);
}

int DynamixelXL430::setVelocityRaw(int32_t vel) {
	return this->write4(104, vel);
}

int DynamixelXL430::setPositionRel(float pos) {
	uint32_t posVal = pos * (this->posMax - this->posMin) + this->posMin;
	return this->setPositionRaw(posVal);
}

int DynamixelXL430::setPwmRel(float pwm) {
	int16_t pwmVal = pwm * this->pwmMax;
	return this->setPwmRaw(pwmVal);
}

int DynamixelXL430::setVelocityRel(float vel) {
	int32_t velVal = vel * this->velMax;
	return this->setVelocityRaw(velVal);
}

int DynamixelXL430::setPosition(float posDeg) {
	uint32_t posVal = posDeg / XL430_DEG_PER_TICK;
	return this->setPositionRaw(posVal);
}

int DynamixelXL430::setPwm(float pwmPercent) {
	int16_t pwmVal = pwmPercent / XL430_PERCENT_PER_TICK;
	return this->setPwmRaw(pwmVal);
}

int DynamixelXL430::setVelocity(float velRpm) {
	int32_t velVal = velRpm / XL430_RPM_PER_TICK;
	return this->setVelocityRaw(velVal);
}

int DynamixelXL430::initMotor() {
	if (this->reboot() != 0)
		return -1;

	sleep_ms(1000);

	uint16_t modelNb = 0;
	if (this->ping(&modelNb) != 0)
		return -1;

	if (modelNb != XL430_MODEL_NB)
		return -1;

	if (this->read2(36, &this->pwmMax) != 0)
		return -1;

	if (this->read4(44, &this->velMax) != 0)
		return -1;

	if (this->read4(48, &this->posMax) != 0)
		return -1;

	if (this->read4(52, &this->posMin) != 0)
		return -1;

	return 0;
}