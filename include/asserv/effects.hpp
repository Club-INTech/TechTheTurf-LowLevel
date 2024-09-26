#pragma  once

#include <asserv/control_loop.hpp>
#include <cstdint>

#define BLINKER_PERIOD 0.25f
#define CENTER_PERIOD 0.15f

class Effects
{
public:
	Effects(ControlLoop *cl, uint8_t left_stop_pin, uint8_t left_blinker_pin, uint8_t right_stop_pin, uint8_t right_blinker_pin, uint8_t center_brake_pin);
	~Effects();

	// Handle all the stuff
	void work();

private:
	ControlLoop *cl;
	uint8_t left_stop_pin, right_stop_pin, center_brake_pin, left_blink_pin, right_blink_pin;

	float blinker_timer;
	float center_timer;
};