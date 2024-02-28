#include <action/comm_action.hpp>
#include <algorithm>

CommAction::CommAction(uint sdaPin, uint sclPin, uint addr, i2c_inst_t *i2c, Elevator *elev, Arm *arm) : Comm(sdaPin, sclPin, addr, i2c) {
	this->cmd = -1;
	this->working = false;
	this->elev = elev;
	this->arm = arm;
}

CommAction::~CommAction() {
}

void CommAction::startWork() {
	this->working = true;
}

void CommAction::finishWork() {
	this->working = false;
}

int CommAction::getCommand() {
	int cmd = this->cmd;

	if (cmd != -1)
		this->cmd = -1;

	return cmd;
}

uint8_t CommAction::getArgumentU8(uint8_t offset) {
	uint8_t data;
	memcpy(&data, &this->args[offset], sizeof(uint8_t));
	return data;
}

uint16_t CommAction::getArgumentU16(uint8_t offset) {
	uint16_t data;
	memcpy(&data, &this->args[offset], sizeof(uint16_t));
	return data;
}

uint32_t CommAction::getArgumentU32(uint8_t offset) {
	uint32_t data;
	memcpy(&data, &this->args[offset], sizeof(uint32_t));
	return data;
}

float CommAction::getArgumentFloat(uint8_t offset) {
	float data;
	memcpy(&data, &this->args[offset], sizeof(float));
	return data;
}

void CommAction::handleCmd(uint8_t *data, size_t size) {
	uint8_t fbyte = data[0];

	uint8_t cmd = fbyte&0xF;
	uint8_t subcmd = (fbyte>>4)&0xF;

	// Floats need to be aligned, can't just cast
	float f1;
	TelemetryBase* telem;

	// Also send CMD to main core, fine to do always because there isn't any overlap in the commands
	if (!this->working) { // Ignore cmds when working for main core
		this->cmd = fbyte;
		memcpy(&this->args, &data[1], size-1);
	}

	switch (cmd) { // Only handle reads from master here, defer everything to main core
		// Elevator control
		case 1:
			if (subcmd == 3) { // Is homed ?
				this->sendDataSize = 1;
				this->sendData[0] = this->elev->isHomed();
			} else if (subcmd == 4) { // Gets position
				this->sendDataSize = sizeof(float);
				f1 = this->elev->getPosition();
				memcpy(&this->sendData[0], &f1, sizeof(float)); 
			}
			break;
		// Arm control
		case 2:
			if (subcmd == 3) { // Is it deployed ?
				this->sendDataSize = 1;
				this->sendData[0] = this->elev->isHomed();
			}
			break;
		// Telem on/off
		case 6: 
			telem = getTelem(subcmd);
			if (!telem)
				break;

			if (data[1])
				telem->start();
			else
				telem->stop();
			break;
		// Ready for next order
		case 10: 
			this->sendDataSize = 1;
			this->sendData[0] = !this->working;
			break;
		default:
			break; 
	}
}