#pragma once

#include <hardware/pio.h>

class Encoder
{
public:
	Encoder(uint pin_a, uint pin_b, bool reversed=false, uint state_machine=0, PIO pio=pio0, int max_step_rate=0);
	~Encoder();

	int32_t getCount();
	float convertRevolutions(int32_t ticks);
	float getRevolutions();
	void reset(int32_t cnt=0);

private:

	void pioInit(int max_step_rate);
	void pioCleanup();

	PIO pio;

	bool reversed;
	uint pinAB;
	uint stateMachine;
};
