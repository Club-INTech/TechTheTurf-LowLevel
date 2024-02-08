#pragma once

#include <pico/stdlib.h>
#include <hardware/uart.h>
#include <stdint.h>

#define SERIAL_HEADER 0x4142
#define COMM_BAUD 1000000

enum MotionControlType {
	torque            = 0x00,     //!< Torque control
	velocity          = 0x01,     //!< Velocity motion control
	angle             = 0x02,     //!< Position/angle motion control
	velocity_openloop = 0x03,
	angle_openloop    = 0x04
};

class CommBG
{
public:
	CommBG(uint8_t uid, uart_inst_t *uart, uint8_t tx, uint8_t rx);
	~CommBG();

	void enable();
	void disable();
	void setTarget(float target);
	void setMotionControl(uint8_t type);
private:
	void sendCmd(uint8_t cmd);
	
	// ID
	uint8_t uid;
	// Pins
	uint8_t tx, rx;
	uart_inst_t *uart;
};