#include <action/dynamixel_motor.hpp>


DynamixelMotor::DynamixelMotor(uint8_t id, float protoVer) {
	this->id = id;
	this->protoVer = protoVer;
}

int DynamixelMotor::bind(DynamixelManager *manager) {
	if (!manager) {
		this->manager = nullptr;
		return 0;
	}
	if (manager->protoVer != this->protoVer)
		return -1;
	this->manager = manager;

	return this->initMotor();
}

int DynamixelMotor::ping() {
	if (!this->manager)
		return -1;
	return this->manager->ping(this->id);
}

int DynamixelMotor::ping(uint16_t *modelNb) {
	if (!this->manager)
		return -1;
	return this->manager->ping(this->id, modelNb);
}

int DynamixelMotor::reboot() {
	if (!this->manager)
		return -1;
	return this->manager->reboot(this->id);
}

int DynamixelMotor::read1(uint8_t address, uint8_t *data) {
	if (!this->manager)
		return -1;
	return this->manager->read1(this->id, address, data);
}

int DynamixelMotor::read2(uint8_t address, uint16_t *data) {
	if (!this->manager)
		return -1;
	return this->manager->read2(this->id, address, data);
}

int DynamixelMotor::read4(uint8_t address, uint32_t *data) {
	if (!this->manager)
		return -1;
	return this->manager->read4(this->id, address, data);
}

int DynamixelMotor::write1(uint8_t address, uint8_t data) {
	if (!this->manager)
		return -1;
	return this->manager->write1(this->id, address, data);
}

int DynamixelMotor::write2(uint8_t address, uint16_t data) {
	if (!this->manager)
		return -1;
	return this->manager->write2(this->id, address, data);
}

int DynamixelMotor::write4(uint8_t address, uint32_t data) {
	if (!this->manager)
		return -1;
	return this->manager->write4(this->id, address, data);
}
