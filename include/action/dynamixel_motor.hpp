#pragma once

#include <action/dynamixel_manager.hpp>

class DynamixelMotor
{
public:
	DynamixelMotor(uint8_t id, float protoVer);

	int bind(DynamixelManager *manager);

	int ping();
	int ping(uint16_t *modelNb);
	int reboot();

	int read1(uint8_t address, uint8_t *data);
	int read2(uint8_t address, uint16_t *data);
	int read4(uint8_t address, uint32_t *data);

	int write1(uint8_t address, uint8_t data);
	int write2(uint8_t address, uint16_t data);
	int write4(uint8_t address, uint32_t data);

	virtual int initMotor() {return 0;}

	uint8_t id;
	float protoVer;
private:
	DynamixelManager *manager;
};
