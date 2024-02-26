#pragma once

#include <pico/stdlib.h>

class Endstop
{
public:
	Endstop(uint8_t pin);
	~Endstop();
	
	bool poll();

private:
	bool state;
	uint8_t pin;
};