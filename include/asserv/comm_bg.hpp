#pragma once


#include <pico/stdlib.h>
#include <pico/sync.h>

#include <hardware/uart.h>
#include <stdint.h>

enum MotionControlType {
	torque            = 0x00,     //!< Torque control
	velocity          = 0x01,     //!< Velocity motion control
	angle             = 0x02,     //!< Position/angle motion control
	velocity_openloop = 0x03,
	angle_openloop    = 0x04
};

// in us
#define RESPONSE_TIMEOUT 2000

class CommBG
{
public:
	CommBG(uint8_t uid, uart_inst_t *uart, uint8_t tx, uint8_t rx);
	~CommBG();

	void enable();
	void disable();
	void setTarget(float target);
	void setMotionControl(uint8_t type);

	int readStats(float *vel, float *current, float *temp, float *vbus);
	float readVel() {float val; readStats(&val, nullptr, nullptr, nullptr); return val;}
	float readCurrent() {float val; readStats(nullptr, &val, nullptr, nullptr); return val;}
	float readTemp() {float val; readStats(nullptr, nullptr, &val, nullptr); return val;}
	float readVbus() {float val; readStats(nullptr, nullptr, nullptr, &val); return val;}
private:
	void sendCmd(uint8_t cmd);
	
	// ID
	uint8_t uid;
	// Pins
	uint8_t tx, rx;
	uart_inst_t *uart;

	mutex_t mutex;
};