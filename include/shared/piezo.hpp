#pragma once

#include <pico/stdlib.h>

class Piezo
{
public:
	Piezo(uint pin, float frequency);
	~Piezo();

	void setFreq(float frequency);
	void setEnable(bool enable);
private:
	uint slice,chan,pin;

	bool enable;
};