#pragma once

#include <pico/stdlib.h>
#include <hardware/uart.h>
#include <stdint.h>

#include <asserv/odrive_enums.hpp>

class CommODrive
{
public:
	CommODrive(uart_inst_t *uart, uint8_t tx, uint8_t rx);
	~CommODrive();

	void clearErrors();
	bool hasErrors();

	void reboot();

	float getVbus();
	// In turns/s
	float getVelocity();
	// In turns
	float getPosition();

	void setAxisState(uint8_t axis, ODrive::AxisState state);
	void setAxisControlMode(uint8_t axis, ODrive::ControlMode mode);

	// Velocity is in turns/s
	void setTargetVelocity(uint8_t axis, float vel);

	// pos in Turns & vel in turns/s 
	void requestFeedback(float *pos, float *vel);
private:
	void uartPrintf(const char* fmt, ...);
	void uartScanf(const char* fmt, ...);

	// Pins
	uint8_t tx, rx;
	uart_inst_t *uart;
};