#include <stdio.h>
#include <encoder.hpp>
#include <pico/stdlib.h>
#include <hardware/pio.h>
#include <hardware/timer.h>
#include <hardware/clocks.h>
#include <hardware/gpio.h>

#include "quadrature_encoder.pio.h"

#define ENCODER_TICK_COUNT 1024

Encoder::Encoder(uint pin_ab, uint state_machine, PIO pio, int max_step_rate) {
	this->pio = pio;
	this->stateMachine = state_machine;
	this->pinAB = pin_ab;
	if (pio_can_add_program(pio, &quadrature_encoder_program))
		pio_add_program(pio, &quadrature_encoder_program);
	pioInit(max_step_rate);
}

Encoder::~Encoder() {
	pioCleanup();
	//pio_remove_program(this->pio, &quadrature_encoder_program, 0);
}

int32_t Encoder::getCount() {
	uint ret;
	int n;

	// if the FIFO has N entries, we fetch them to drain the FIFO,
	// plus one entry which will be guaranteed to not be stale
	n = pio_sm_get_rx_fifo_level(this->pio, this->stateMachine) + 1;
	while (n > 0) {
		ret = pio_sm_get_blocking(this->pio, this->stateMachine);
		n--;
	}
	return ret;
}

float Encoder::convertRevolutions(int32_t ticks) {
	return ((float)ticks)/((float)ENCODER_TICK_COUNT);
}

float Encoder::getRevolutions() {
	return convertRevolutions(getCount());
}

// max_step_rate is used to lower the clock of the state machine to save power
// if the application doesn't require a very high sampling rate. Passing zero
// will set the clock to the maximum

void Encoder::pioInit(int max_step_rate)
{
	pio_sm_set_consecutive_pindirs(this->pio, this->stateMachine, this->pinAB, 2, false);
	gpio_pull_up(this->pinAB);
	gpio_pull_up(this->pinAB + 1);

	pio_sm_config cfg = quadrature_encoder_program_get_default_config(0);

	sm_config_set_in_pins(&cfg, this->pinAB); // for WAIT, IN
	sm_config_set_jmp_pin(&cfg, this->pinAB); // for JMP
	// shift to left, autopull disabled
	sm_config_set_in_shift(&cfg, false, false, 32);
	// don't join FIFO's
	sm_config_set_fifo_join(&cfg, PIO_FIFO_JOIN_NONE);

	// passing "0" as the sample frequency,
	if (max_step_rate == 0) {
		sm_config_set_clkdiv(&cfg, 1.0);
	} else {
		// one state machine loop takes at most 10 cycles
		float div = (float)clock_get_hz(clk_sys) / (10 * max_step_rate);
		sm_config_set_clkdiv(&cfg, div);
	}

	pio_sm_init(this->pio, this->stateMachine, 0, &cfg);
	pio_sm_set_enabled(this->pio, this->stateMachine, true);
}

void Encoder::pioCleanup()
{
	gpio_disable_pulls(this->pinAB);
	gpio_disable_pulls(this->pinAB + 1);

	pio_sm_set_enabled(this->pio, this->stateMachine, false);
}