#include "comm_bg.hpp"

CommBG::CommBG(uint8_t uid, HardwareSerial &ser, uint8_t rx, uint8_t tx, FOCMotor &mot) {
	this->uid = uid;
	this->ser = &ser;
	this->mot = &mot;
	this->rx = rx;
	this->tx = tx;
}

CommBG::~CommBG() {

}

void CommBG::begin() {
	this->stopTx();
	pinMode(A_TEMPERATURE, INPUT);
	pinMode(A_VBUS, INPUT);
	analogReadResolution(12);
}

void CommBG::startTx() {
	pinMode(this->tx, OUTPUT);
}

void CommBG::stopTx() {
	pinMode(this->tx, INPUT);
}

void CommBG::sendCmd(uint8_t cmd) {
	uint16_t val = ((SERIAL_HEADER&0xFF) << 8) | ((SERIAL_HEADER>>8) & 0xFF);
	this->ser->write((uint8_t*)&val, 2);
	this->ser->write(this->uid);
	this->ser->write(cmd);
}

void CommBG::work() {
	while (this->ser->available() >= 3) {
		uint8_t by = this->ser->read();
		this->last = by | (this->last << 8);
		if (this->last != SERIAL_HEADER)
			continue;

		by = this->ser->read();
		if (by != this->uid)
			continue;
		this->ser->print("UID: ");
		this->ser->println(by);

		by = this->ser->read();
		this->ser->print("CMD: ");
		this->ser->println(by);
		float tmpf;
		uint32_t tmpi;
		switch (by) { // CMD
			case 0x0: // Disable
				this->mot->disable();
				break;
			case 0x1: // Enable
				this->mot->enable();
				break;
			case 0x2: // Set Target
				this->ser->readBytes((uint8_t*)&tmpf, sizeof(float)); 
				//this->ser->print("ST: ");
				//this->ser->println(tmpf);
				this->mot->target = tmpf;
				break;
			case 0x3: // Change ctrl mode
				this->mot->controller = (MotionControlType)this->ser->read();
				//this->ser->print("CTRL: ");
				this->ser->println(this->mot->controller);
				break;
			case 0x4: // Get stats
				this->startTx();
				this->sendCmd(0x84);
				tmpf = this->mot->shaftVelocity();
				tmpi = analogRead(A_TEMPERATURE);
				this->ser->write((uint8_t*)&tmpf, sizeof(float));
				this->ser->write((uint8_t*)&tmpi, sizeof(uint32_t));
				tmpi = analogRead(A_VBUS);
				this->ser->write((uint8_t*)&tmpi, sizeof(uint32_t));
				break;
			case 0x7F: // Stop TX
				this->stopTx();
				break;
		}
	}
}