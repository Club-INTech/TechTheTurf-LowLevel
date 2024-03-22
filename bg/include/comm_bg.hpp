#pragma once

#include <stdint.h>
#include <HardwareSerial.h>
#include <SimpleFOC.h>

#define SERIAL_HEADER 0x4142

class CommBG
{
public:
	CommBG(uint8_t uid, HardwareSerial &ser, FOCMotor &mot);
	~CommBG();

	void work();
private:
	// Last bys
	uint16_t last;

	// ID
	uint8_t uid;
	HardwareSerial *ser;
	FOCMotor *mot;
};