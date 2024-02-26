#pragma once

#include <shared/comm.hpp>

class CommAction : public Comm
{
public:
	CommAction(uint sdaPin, uint sclPin, uint addr, i2c_inst_t *i2c);
	~CommAction();
	
	void handleCmd(uint8_t *data, size_t size);

	bool start;

private:
};