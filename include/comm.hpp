#pragma once

#include <hardware/i2c.h>
#include <pico/i2c_slave.h>

#include <control_loop.hpp>

// 100Khz
#define I2C_BAUDRATE 100000
#define MAX_DATA_SIZE 64

class Comm
{
public:
	Comm(uint sdaPin, uint sclPin, uint addr, i2c_inst_t *i2c, ControlLoop *cl);
	~Comm();
	
	void slaveHandler(i2c_slave_event_t event);
private:

	void handleCmd(uint8_t *data, size_t size);

	void i2cInit(uint8_t address);
	void i2cDeinit();

	void resetCmd();
	void resetRecvCmd();
	void resetSendCmd();

	// Temporary buffers to send the response
	size_t sendDataSize;
	uint8_t sendData[MAX_DATA_SIZE];

	// Temporary buffers to receive the command
	size_t recvDataSize;
	uint8_t recvData[MAX_DATA_SIZE];

	uint sdaPin;
	uint sclPin;
	i2c_inst_t *i2c;

	ControlLoop *cl;
};