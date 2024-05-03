#include "comm_bg.hpp"

CommBG::CommBG(uint8_t uid, HardwareSerial &ser, uint8_t rx, uint8_t tx, BLDCMotor &mot, LowsideCurrentSense &currSense) {
	this->uid = uid;
	this->ser = &ser;
	this->mot = &mot;
	this->rx = rx;
	this->tx = tx;
	this->currSense = &currSense;
}

CommBG::~CommBG() {

}

void CommBG::begin() {
	this->stopTx();
}

void CommBG::startTx() {
	pin_function(digitalPinToPinName(this->tx), STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF7_USART2));
}

void CommBG::stopTx() {
	pin_function(digitalPinToPinName(this->tx), STM_PIN_DATA(STM_MODE_INPUT, GPIO_NOPULL, 0));
}

void CommBG::sendCmd(uint8_t cmd) {
	uint16_t val = ((SERIAL_HEADER&0xFF) << 8) | ((SERIAL_HEADER>>8) & 0xFF);
	this->ser->write((uint8_t*)&val, 2);
	this->ser->write(this->uid);
	this->ser->write(cmd);
}

// V -> Celsius
static float Ntc2TempV(float ADCVoltage)
{
	// Formula: https://www.giangrandi.org/electronics/ntc/ntc.shtml
	const float ResistorBalance = 4700.0;
	const float Beta  = 3425.0F;
	const float RoomTempI = 1.0F/298.15F; //[K]
	const float Rt = ResistorBalance * ((3.3F / ADCVoltage)-1);
	const float R25 = 10000.0F;

	float T = 1.0F/((log(Rt/R25)/Beta)+RoomTempI);
	T = T - 273.15;

	return T;
}

float CommBG::getPower() {
	PhaseCurrent_s current = this->currSense->getPhaseCurrents();
	ABCurrent_s ABcurrent = this->currSense->getABCurrents(current);
	float apow = ABcurrent.alpha * this->mot->Ualpha;
	float bpow = ABcurrent.beta * this->mot->Ubeta;
	return _sqrt(apow*apow + bpow*bpow);
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
		//this->ser->print("UID: ");
		//this->ser->println(by);

		by = this->ser->read();
		//this->ser->print("CMD: ");
		//this->ser->println(by);
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
			case 0x4: // Get stats
				this->startTx();
				this->sendCmd(0x84);
				tmpf = this->mot->shaftVelocity();
				this->ser->write((uint8_t*)&tmpf, sizeof(float));
				tmpf = this->currSense->getDCCurrent();//this->getPower();
				this->ser->write((uint8_t*)&tmpf, sizeof(float));
				tmpf =  Ntc2TempV(_readADCVoltageInline(A_TEMPERATURE, this->currSense->params));
				this->ser->write((uint8_t*)&tmpf, sizeof(float));
				tmpf = _readADCVoltageInline(A_VBUS, this->currSense->params) * ((18.0f+169.0f)/18.0f);
				this->ser->write((uint8_t*)&tmpf, sizeof(float));
				break;
			case 0x7F: // Stop TX
				this->stopTx();
				break;
		}
	}
}