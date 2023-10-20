#pragma once

#include <hardware/i2c.h>
#include <pico/i2c_slave.h>

// 100Khz
#define I2C_BAUDRATE 100000
#define MAX_DATA_SIZE 64

class Comm
{
public:
	Comm(uint sdaPin, uint sclPin, uint addr, i2c_inst_t *i2c);
	~Comm();
	
	void slaveHandler(i2c_inst_t *i2c, i2c_slave_event_t event);
private:

	void handleCmd(uint8_t *data, size_t size);


	void resetCmd();

	size_t respDataSize;
	uint8_t respData[MAX_DATA_SIZE];

	// Temporary buffers to parse the command
	size_t cmdDataSize;
	uint8_t cmdData[MAX_DATA_SIZE];
	bool finishedRecv;

	uint sdaPin;
	uint sclPin;
};