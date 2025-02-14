#include <cmath>
#include <cstdint>
#include <shared/ina236.hpp>

INA236::INA236(i2c_inst_t *inst, uint8_t sda, uint8_t scl, uint8_t addr, float shuntResistance, float maxCurrent, uint baudrate, bool pullUp, uint timeoutUs)
 : I2CDevice(inst, sda, scl, addr, baudrate, pullUp, true, timeoutUs), maxCurrent(maxCurrent), shuntRes(shuntResistance) {
 	reset();
}

// Will erase all config, will only set back the calibration value
void INA236::reset() {
	writeU16(0x0, 0x8000);
	busy_wait_us(100);
	readConfig();
	updateCalibration();
}

// Ohms
void INA236::setShuntResistance(float res) {
	this->shuntRes = res;
	updateCalibration();
}
// Amps
void INA236::setMaxCurrent(float curr) {
	this->maxCurrent = curr;
	updateCalibration();
}

void INA236::setAdcScale(INA236Scale scale, bool apply) {
	this->config = (this->config & ~(0b1 << 12)) | (uint16_t(scale) << 12);
	if (apply)
		writeConfig();
}
void INA236::setAveraging(INA236Avg avg, bool apply) {
	this->config = (this->config & ~(0b111 << 9)) | (uint16_t(avg) << 9);
	if (apply)
		writeConfig();
}
void INA236::setBusCT(INA236CT ct, bool apply) {
	this->config = (this->config & ~(0b111 << 6)) | (uint16_t(ct) << 6);
	if (apply)
		writeConfig();
}
void INA236::setShuntCT(INA236CT ct, bool apply) {
	this->config = (this->config & ~(0b111 << 3)) | (uint16_t(ct) << 3);
	if (apply)
		writeConfig();
}
void INA236::setMode(INA236Mode mode, bool apply) {
	this->config = (this->config & ~(0b111 << 0)) | (uint16_t(mode) << 0);
	if (apply)
		writeConfig();
}

// All in one set config
void INA236::setConfig(INA236Scale scale, INA236Avg avg, INA236CT busCt, INA236CT shuntCt, INA236Mode mode) {
	setAdcScale(scale, false);
	setAveraging(avg, false);
	setBusCT(busCt, false);
	setShuntCT(shuntCt, false);
	setMode(mode, true);
}

float INA236::readShuntVoltage() {
	uint16_t voltage = 0;
	readU16(0x1, &voltage);
	float voltageLsb = (this->config >> 12) & 1 ? 625.0e-9 : 2.5e-6;
	return  voltage * voltageLsb;
}
float INA236::readBusVoltage() {
	uint16_t voltage = 0;
	readU16(0x2, &voltage);
	return voltage * 1.6e-3;
}
float INA236::readPower() {
	uint16_t power = 0;
	readU16(0x3, &power);
	return 32.0f * power * this->currentLsb;

}
float INA236::readCurrent() {
	uint16_t current = 0;
	readU16(0x4, &current);
	return current * this->currentLsb;
}

uint16_t INA236::readManufacturerID() {
	uint16_t val = 0;
	readU16(0x3E, &val);
	return val;
}

uint16_t INA236::readDeviceID() {
	uint16_t val = 0;
	readU16(0x3F, &val);
	return val;
}

void INA236::updateCalibration() {
	this->currentLsb = this->maxCurrent / 32768.0f;
	
	float shuntCalFlt = 0.00512f / (this->currentLsb * this->shuntRes);
	if ((this->config >> 12) & 1) // ADCRANGE = 1
		shuntCalFlt /= 4;

	writeU16(0x5, std::round(shuntCalFlt));
}

void INA236::writeConfig() {
	updateCalibration(); // Needed if the scale changed
	writeU16(0x0, this->config);
}

void INA236::readConfig() {
	readU16(0x0, &this->config);
}