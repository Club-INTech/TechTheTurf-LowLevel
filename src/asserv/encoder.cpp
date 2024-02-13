#include <stdio.h>
#include <pico/stdlib.h>
#include <hardware/pio.h>
#include <hardware/timer.h>
#include <hardware/clocks.h>
#include <hardware/gpio.h>
#include <math.h>

#include <asserv/encoder.hpp>

#include "quadrature_encoder.pio.h"

#define ENCODER_TICK_COUNT (1024*4)

Encoder::Encoder(uint pin_a, uint pin_b, bool reversed, uint state_machine, PIO pio, int max_step_rate) {
	this->pio = pio;
	this->stateMachine = state_machine;
	assert(abs(((int)pin_a)-((int)pin_b)) == 1);
	this->reversed = reversed;
	if (pin_a > pin_b) {
		this->reversed ^= true;
		pin_a = pin_b;
	}
	this->pinAB = pin_a;

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
	return this->reversed ? -ret : ret;
}

float Encoder::convertRevolutions(int32_t ticks) {
	return ((float)ticks)/((float)ENCODER_TICK_COUNT);
}

float Encoder::getRevolutions() {
	return convertRevolutions(getCount());
}

void Encoder::reset(int32_t cnt) {
	pio_sm_set_enabled(this->pio, this->stateMachine, false);
	pio_sm_drain_tx_fifo(this->pio, this->stateMachine);
	// From https://github.com/zapta/simple_stepper_motor_analyzer/blob/master/platformio/src/display/tft_driver.cpp#L65
	static const uint instr_shift = pio_encode_in(pio_y, 4);
	static const uint instr_mov = pio_encode_mov(pio_y, pio_isr);
	for (int i = 7; i >= 0; i--) {
		const uint32_t nibble = (cnt >> (i * 4)) & 0xf;
		pio_sm_exec(this->pio, this->stateMachine, pio_encode_set(pio_y, nibble));
		pio_sm_exec(this->pio, this->stateMachine, instr_shift);
	}
	pio_sm_exec(this->pio, this->stateMachine, instr_mov);
	pio_sm_set_enabled(this->pio, this->stateMachine, true);
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