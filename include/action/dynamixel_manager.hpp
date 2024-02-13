#pragma once

#include <DynamixelSDK.h>

#include <hardware/uart.h>

struct PortHandlerInfo {
	uint8_t uart_id;
	uint8_t tx;
	uint8_t rx;
};

class DynamixelManager
{
public:
	DynamixelManager(uart_inst_t *uart, uint8_t tx, uint8_t rx,  uint32_t baudrate=57600, float protoVer=2.0);
	~DynamixelManager();

	int ping(uint8_t id);
	int ping(uint8_t id, uint16_t *modelNb);
	int reboot(uint8_t id);

	int read1(uint8_t id, uint8_t address, uint8_t *data);
	int read2(uint8_t id, uint8_t address, uint16_t *data);
	int read4(uint8_t id, uint8_t address, uint32_t *data);

	int write1(uint8_t id, uint8_t address, uint8_t data);
	int write2(uint8_t id, uint8_t address, uint16_t data);
	int write4(uint8_t id, uint8_t address, uint32_t data);

	int lastError;
	uint8_t lastHardwareError;
	const char *lastErrorString;

	float protoVer;

private:
	int errorCheck(); 

	uart_inst_t *uart;

	uint8_t tx,rx;
	uint8_t baudrate;

	dynamixel::PacketHandler *handler;
	dynamixel::PortHandler *port;
};