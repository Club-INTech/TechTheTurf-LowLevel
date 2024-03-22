#include "comm_bg.hpp"

CommBG::CommBG(uint8_t uid, HardwareSerial &ser, FOCMotor &mot) {
	this->uid = uid;
	this->ser = &ser;
	this->mot = &mot;
}

CommBG::~CommBG() {

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
		}
	}
}