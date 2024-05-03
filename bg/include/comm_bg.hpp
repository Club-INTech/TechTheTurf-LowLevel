#pragma once

#include <stdint.h>
#include <HardwareSerial.h>
#include <SimpleFOC.h>

#define SERIAL_HEADER 0x4142

class CommBG
{
public:
	CommBG(uint8_t uid, HardwareSerial &ser, uint8_t rx, uint8_t tx, BLDCMotor &mot, LowsideCurrentSense &currSense);
	~CommBG();

	void begin();

	void work();
private:

	float getPower();

	void startTx();
	void stopTx();
	void sendCmd(uint8_t cmd);

	// Last bys
	uint16_t last;

	// ID
	uint8_t uid;
	HardwareSerial *ser;
	uint8_t rx, tx;
	BLDCMotor *mot;
	LowsideCurrentSense *currSense;
};