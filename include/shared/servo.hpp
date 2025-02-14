#pragma once

#include <pico/stdlib.h>

class Servo
{
public:
	Servo(uint pin, double freq=50.0, bool centered=true, double min=0.5e-3, double max=2.5e-3);
	~Servo();

	void setFreq(double freq);
	void setCentered(bool centered);
	void setRange(double min, double max);

	void setRaw(double us);
	// -1 to 1 if centered, 0 to 1 if not
	void setValue(float val);

	// Disable output
	void disable();

private:

	void updatePWM();

	double frequency;
	double minTime,maxTime;
	bool centered;

	double realPeriod;
	uint16_t minValue,maxValue;

	uint slice,chan,pin;
};