#pragma once

#include <pico/stdlib.h>

#define PICO_PIN_COUNT 40

class HCSR04
{
public:
	HCSR04(uint8_t trig, uint8_t echo);
	~HCSR04();

	// Only blocks for the duration of the trigger 10us
	void trigger();
	// Doesn't block, can return old values
	float getLastDistance();

	bool hasNewDist();

	// Blocking
	float getDistance();

private:
	static void irqHandler();
	static HCSR04* irqHandles[PICO_PIN_COUNT];

	uint8_t trig, echo;
	float lastDist;
	volatile bool newValue;

	volatile uint64_t riseTime;
};