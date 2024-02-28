#pragma once

#include <pico/stdlib.h>

class Pump
{
public:
	Pump(uint8_t pin);
	~Pump();

	bool getState();
	void setEnable(bool en);

	void enable();
	void disable();
	
private:

	bool state;

	uint8_t pin;
};