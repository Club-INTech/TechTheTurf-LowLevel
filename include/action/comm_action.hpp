#pragma once

#include <shared/comm.hpp>
#include <action/elevator.hpp>
#include <action/arm.hpp>

class CommAction : public Comm
{
public:
	CommAction(uint sdaPin, uint sclPin, uint addr, i2c_inst_t *i2c, Elevator *elev, Arm *arm);
	~CommAction();
	
	void handleCmd(uint8_t *data, size_t size);

	void startWork();
	void finishWork();

	int getCommand();
	uint8_t getArgumentU8(uint8_t offset);
	uint16_t getArgumentU16(uint8_t offset);
	uint32_t getArgumentU32(uint8_t offset);
	float getArgumentFloat(uint8_t offset);

private:

	Elevator *elev;
	Arm *arm;

	int cmd;
	uint8_t args[4*2]; // Max 2 floats

	bool working;
};