#pragma once

#include <shared/comm.hpp>

#include <asserv/control_loop.hpp>
#include <shared/telemetry.hpp>

class CommAsserv : public Comm
{
public:
	CommAsserv(uint sdaPin, uint sclPin, uint addr, i2c_inst_t *i2c, ControlLoop *cl);
	~CommAsserv();
	
	void handleCmd(uint8_t *data, size_t size);

private:
	ControlLoop *cl;
};