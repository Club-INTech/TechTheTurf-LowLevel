#pragma once

#include <stdint.h>

class LDR
{
public:
	LDR(uint8_t pin, float r_base, float coeff, float exponent);
	~LDR();

	float readLux();
	
private:
	uint8_t pin;
	float r_base, coeff, exponent;
};