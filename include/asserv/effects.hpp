#pragma  once

#include <asserv/control_loop.hpp>
#include <cstdint>

#define BLINKER_PERIOD 0.25f

class Effects
{
public:
	Effects(ControlLoop *cl, uint8_t left_stop_pin, uint8_t left_blinker_pin, uint8_t right_stop_pin, uint8_t right_blinker_pin);
	~Effects();

	// Handle all the stuff
	void work();

private:
	ControlLoop *cl;
	uint8_t left_stop_pin, right_stop_pin, left_blink_pin, right_blink_pin;

	float oldDst, oldTheta;

	float blinker_timer;
};