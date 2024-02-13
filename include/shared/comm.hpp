#pragma once

#include <hardware/i2c.h>
#include <pico/i2c_slave.h>

#include <shared/telemetry.hpp>

// 1Mhz
#define I2C_BAUDRATE 1000000
#define MAX_DATA_SIZE 64

#define MAX_TELEMETRY_NB 16

class Comm
{
public:
	Comm(uint sdaPin, uint sclPin, uint addr, i2c_inst_t *i2c);
	~Comm();

	void slaveHandler(i2c_slave_event_t event);

	TelemetryBase* getTelem(uint8_t idx);
	void addTelem(uint8_t idx, TelemetryBase *telem);
	void clearTelems();

	void work();
	virtual void handleCmd(uint8_t *data, size_t size) = 0;

protected:
	void i2cInit(uint8_t address);
	void i2cDeinit();

	void resetCmd();
	void resetRecvCmd();
	void resetSendCmd();

	// Store reference to all telemetries that we have to handle in serial
	TelemetryBase* telems[MAX_TELEMETRY_NB];

	// Temporary buffer for Serial telemetry
	uint8_t serialBuffer[MAX_DATA_SIZE];

	// Temporary buffer to send the response
	size_t sendDataSize;
	uint8_t sendData[MAX_DATA_SIZE];

	// Temporary buffer to receive the command
	size_t recvDataSize;
	uint8_t recvData[MAX_DATA_SIZE];

	uint sdaPin;
	uint sclPin;
	i2c_inst_t *i2c;
};