#include <asserv/encoder.hpp>

#include <math.h>
#include <hardware/clocks.h>
#include <hardware/gpio.h>
#include <hardware/sync.h>
#include <pico/stdlib.h>
#include <hardware/pio.h>
#include <hardware/timer.h>
#include <pico/divider.h>
#include <cstdio>
#include <cstring>

#include "quadrature_encoder_substep.pio.h"

#define ENCODER_TICK_COUNT (1024*256)

Encoder::Encoder(uint pin_a, uint pin_b, bool reversed, uint state_machine, PIO pio) {
	this->pio = pio;
	this->stateMachine = state_machine;
	assert(abs(((int)pin_a)-((int)pin_b)) == 1);
	this->reversed = reversed;
	if (pin_a > pin_b) {
		this->reversed ^= true;
		pin_a = pin_b;
	}
	this->pinAB = pin_a;

	if (pio_can_add_program(pio, &quadrature_encoder_substep_program))
		pio_add_program(pio, &quadrature_encoder_substep_program);

	pioInit();


	// start with equal phase size calibration
	this->calibration_data[0] = 0;
	this->calibration_data[1] = 64;
	this->calibration_data[2] = 128;
	this->calibration_data[3] = 192;

	this->idle_stop_samples = 3;

	// start "stopped" so that we don't use stale data to compute speeds
	this->stopped = 1;
	this->speed = 0;

	// cache the PIO cycles per us
	this->clocks_per_us = (clock_get_hz(clk_sys) + 500000) / 1000000;

	int forward;
	// initialize the "previous state"
	readPioData(&this->raw_step, &this->prev_step_us, &this->prev_trans_us, &forward);

	this->position = getStepStartTransitionPos(this->raw_step) + 32;
}

Encoder::~Encoder() {
	pioCleanup();
	//pio_remove_program(this->pio, &quadrature_encoder_program, 0);
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

	static const uint instr_shift2 = pio_encode_in(pio_x, 4);
	static const uint instr_mov2 = pio_encode_mov(pio_x, pio_isr);
	for (int i = 7; i >= 0; i--) {
		pio_sm_exec(this->pio, this->stateMachine, pio_encode_set(pio_x, 0));
		pio_sm_exec(this->pio, this->stateMachine, instr_shift2);
	}
	pio_sm_exec(this->pio, this->stateMachine, instr_mov2);

	pio_sm_set_enabled(this->pio, this->stateMachine, true);

	this->stopped = 1;

	int forward;
	readPioData(&this->raw_step, &this->prev_step_us, &this->prev_trans_us, &forward);

	this->position = getStepStartTransitionPos(this->raw_step) + 32;
}

float Encoder::convertRevolutions(int32_t ticks) {
	return ((float)ticks)/((float)ENCODER_TICK_COUNT);
}

int32_t Encoder::getCount() {
	return this->reversed ? -this->position : this->position;
}

int32_t Encoder::getSpeedCount() {
	return this->reversed ? -this->speed : this->speed;
}

float Encoder::getRevolutions() {
	return convertRevolutions(getCount());
}

float Encoder::getSpeedRev() {
	return convertRevolutions(getSpeedCount());
}

// "substep" version low-level interface
//
// note: user code should use the high level functions in quadrature_encoder.c
// and not call these directly

// initialize the PIO state and the substep_state_t structure that keeps track
// of the encoder state
void Encoder::pioInit()
{
	uint pin_state, position, ints;
	pio_gpio_init(this->pio, this->pinAB);
	pio_gpio_init(this->pio, this->pinAB + 1);

	pio_sm_set_consecutive_pindirs(this->pio, this->stateMachine, this->pinAB, 2, false);
	gpio_pull_up(this->pinAB);
	gpio_pull_up(this->pinAB + 1);

	pio_sm_config c = quadrature_encoder_substep_program_get_default_config(0);
	sm_config_set_in_pins(&c, this->pinAB); // for WAIT, IN
	// shift to left, auto-push at 32 bits
	sm_config_set_in_shift(&c, false, true, 32);
	sm_config_set_out_shift(&c, true, false, 32);
	// don't join FIFO's
	sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_NONE);

	// always run at sysclk, to have the maximum possible time resolution
	sm_config_set_clkdiv(&c, 1.0);

	pio_sm_init(this->pio, this->stateMachine, 0, &c);

	// set up status to be rx_fifo < 1
	this->pio->sm[this->stateMachine].execctrl = ((this->pio->sm[this->stateMachine].execctrl & 0xFFFFFF80) | 0x12);

	// init the state machine according to the current phase. Since we are
	// setting the state running PIO instructions from C state, the encoder may
	// step during this initialization. This should not be a problem though,
	// because as long as it is just one step, the state machine will update
	// correctly when it starts. We disable interrupts anyway, to be safe
	ints = save_and_disable_interrupts();

	pin_state = (gpio_get_all() >> this->pinAB) & 3;

	// to setup the state machine, we need to set the lower 2 bits of OSR to be
	// the negated pin state
	pio_sm_exec(this->pio, this->stateMachine, pio_encode_set(pio_y, ~pin_state));
	pio_sm_exec(this->pio, this->stateMachine, pio_encode_mov(pio_osr, pio_y));

	// also set the Y (current step) so that the lower 2 bits of Y have a 1:1
	// mapping to the current phase (input pin state). That simplifies the code
	// to compensate for differences in encoder phase sizes:
	switch (pin_state) {
		case 0: position = 0; break;
		case 1: position = 3; break;
		case 2: position = 1; break;
		case 3: position = 2; break;
	} 
	pio_sm_exec(this->pio, this->stateMachine, pio_encode_set(pio_y, position));

	pio_sm_set_enabled(this->pio, this->stateMachine, true);
	
	restore_interrupts(ints);
}

void Encoder::pioCleanup()
{
	gpio_disable_pulls(this->pinAB);
	gpio_disable_pulls(this->pinAB + 1);

	pio_sm_set_enabled(this->pio, this->stateMachine, false);
}

void Encoder::fetchCounts(uint *step, int *cycles, uint *us)
{
	int i, pairs;
	uint ints;
	
	pairs = pio_sm_get_rx_fifo_level(this->pio, this->stateMachine) >> 1;

	// read all data with interrupts disabled, so that there can not be a
	// big time gap between reading the PIO data and the current us
	ints = save_and_disable_interrupts();
	for (i = 0; i < pairs + 1; i++) {
		*cycles = pio_sm_get_blocking(this->pio, this->stateMachine);
		*step = pio_sm_get_blocking(this->pio, this->stateMachine);
	}
	*us = time_us_32();
	restore_interrupts(ints);
}


// internal helper functions (not to be used by user code)

void Encoder::readPioData(uint *step, uint *step_us, uint *transition_us, int *forward)
{
	int cycles;

	// get the raw data from the PIO state machine
	fetchCounts(step, &cycles, step_us);

	// when the PIO program detects a transition, it sets cycles to either zero
	// (when step is incrementing) or 2^31 (when step is decrementing) and keeps
	// decrementing it on each 13 clock loop. We can use this information to get
	// the time and direction of the last transition
	if (cycles < 0) {
		cycles = -cycles;
		*forward = 1;
	} else {
		cycles = 0x80000000 - cycles;
		*forward = 0;
	}
	*transition_us = *step_us - ((cycles * 13) / this->clocks_per_us);
}

// get the sub-step position of the start of a step
uint Encoder::getStepStartTransitionPos(uint step)
{
	return ((step << 6) & 0xFFFFFF00) | this->calibration_data[step & 3];
}

// compute speed in "sub-steps per 2^20 us" from a delta substep position and
// delta time in microseconds. This unit is cheaper to compute and use, so we
// only convert to "sub-steps per second" once per update, at most
static int substep_calc_speed(int delta_substep, int delta_us)
{
	return ((int64_t) delta_substep << 20) / delta_us;
}

// main functions to be used by user code

// read the PIO data and update the speed / position estimate
void Encoder::update()
{
	uint step, step_us, transition_us, transition_pos, low, high;
	int forward, speed_high, speed_low;

	// read the current encoder state from the PIO
	readPioData(&step, &step_us, &transition_us, &forward);

	// from the current step we can get the low and high boundaries in substeps
	// of the current position
	low = getStepStartTransitionPos(step);
	high = getStepStartTransitionPos(step + 1);

	// if we were not stopped, but the last transition was more than
	// "idle_stop_samples" ago, we are stopped now
	if (step == this->raw_step)
		this->idle_stop_sample_count++;
	else
		this->idle_stop_sample_count = 0;

	if (!this->stopped && this->idle_stop_sample_count >= this->idle_stop_samples) {
		this->speed = 0;
		this->speed_2_20 = 0;
		this->stopped = 1;
	}

	// when we are at a different step now, there is certainly a transition
	if (this->raw_step != step) {
		// the transition position depends on the direction of the move
		transition_pos = forward ? low : high;

		// if we are not stopped, that means there is valid previous transition
		// we can use to estimate the current speed
		if (!this->stopped)
			this->speed_2_20 = substep_calc_speed(transition_pos - this->prev_trans_pos, transition_us - this->prev_trans_us);

		// if we have a transition, we are not stopped now
		this->stopped = 0;
		// save the timestamp and position of this transition to use later to
		// estimate speed
		this->prev_trans_pos = transition_pos;
		this->prev_trans_us = transition_us;
	}

	// if we are stopped, speed is zero and the position estimate remains
	// constant. If we are not stopped, we have to update the position and speed
	if (!this->stopped) {
		// although the current step doesn't give us a precise position, it does
		// give boundaries to the position, which together with the last
		// transition gives us boundaries for the speed value. This can be very
		// useful especially in two situations:
		// - we have been stopped for a while and start moving quickly: although
		//   we only have one transition initially, the number of steps we moved
		//   can already give a non-zero speed estimate
		// - we were moving but then stop: without any extra logic we would just
		//   keep the last speed for a while, but we know from the step
		//   boundaries that the speed must be decreasing

		// if there is a transition between the last sample and now, and that
		// transition is closer to now than the previous sample time, we should
		// use the slopes from the last sample to the transition as these will
		// have less numerical issues and produce a tighter boundary
		if (this->prev_trans_us > this->prev_step_us && 
			(int)(this->prev_trans_us - this->prev_step_us) > (int)(step_us - this->prev_trans_us)) {
			speed_high = substep_calc_speed(this->prev_trans_pos - this->prev_low, this->prev_trans_us - this->prev_step_us);
			speed_low = substep_calc_speed(this->prev_trans_pos - this->prev_high, this->prev_trans_us - this->prev_step_us);
		} else {
			// otherwise use the slopes from the last transition to now
			speed_high = substep_calc_speed(high - this->prev_trans_pos, step_us - this->prev_trans_us);
			speed_low = substep_calc_speed(low - this->prev_trans_pos, step_us - this->prev_trans_us);
		}
		// make sure the current speed estimate is between the maximum and
		// minimum values obtained from the step slopes
		if (this->speed_2_20 > speed_high)
			this->speed_2_20 = speed_high;
		if (this->speed_2_20 < speed_low)
			this->speed_2_20 = speed_low;

		// convert the speed units from "sub-steps per 2^20 us" to "sub-steps
		// per second"
		this->speed = (this->speed_2_20 * 62500LL) >> 16;

		// estimate the current position by applying the speed estimate to the
		// most recent transition
		this->position = this->prev_trans_pos + (((int64_t)this->speed_2_20 * (step_us - transition_us)) >> 20);

		// make sure the position estimate is between "low" and "high", as we
		// can be sure the actual current position must be in this range
		if ((int)(this->position - high) > 0)
			this->position = high;
		else if ((int)(this->position - low) < 0)
			this->position = low;
	}

	// save the current values to use on the next sample
	this->prev_low = low;
	this->prev_high = high;
	this->raw_step = step;
	this->prev_step_us = step_us;
}


// function to measure the difference between the different steps on the encoder
void Encoder::calibratePhase()
{
#define sample_count  1024
//#define SHOW_ALL_SAMPLES
#ifdef SHOW_ALL_SAMPLES
	static int result[sample_count];
	int i;
#endif
	int index, cycles, clocks_per_us, calib[4];
	uint cur_us, last_us, step_us, step, last_step;
	int64_t sum[4], total;

	memset(sum, 0, sizeof(sum));

	clocks_per_us = (clock_get_hz(clk_sys) + 500000) / 1000000;

	// keep reading the PIO state in a tight loop to get all steps and use the
	// transition measures of the PIO code to measure the time of each step
	last_step = -10;
	index = -10;
	while (index < sample_count) {

		fetchCounts(&step, &cycles, &step_us);

		// wait until we have a transition
		if (step == last_step)
			continue;

		// synchronize the index with the lower 2 bits of the current step
		if (index < 0 && index > -4 && (step & 3) == 1)
			index = 0;

		// convert the "time since last transition" to an absolute microsecond
		// timestamp
		if (cycles > 0) {
			printf("error: expected forward motion\n");
			return;
		}
		cur_us = step_us + (cycles * 13) / clocks_per_us;

		// if the index is already synchronized, use the step size
		if (index >= 0) {
#ifdef SHOW_ALL_SAMPLES
			result[index] = cur_us - last_us;
#endif
			sum[(step - 1) & 3] += cur_us - last_us;
		}
		index++;

		last_step = step;
		last_us = cur_us;
	}

#ifdef SHOW_ALL_SAMPLES
	printf("full sample table:\n");
	for (i = 0; i < sample_count; i++) {
		printf("%d ", result[i]);
		if ((i & 3) == 3)
			printf("\n");
	}
#endif

	// scale the sizes to a total of 256 to be used as sub-steps
	total = sum[0] + sum[1] + sum[2] + sum[3];
	calib[0] = (sum[0] * 256 + total / 2) / total;
	calib[1] = ((sum[0] + sum[1]) * 256 + total / 2) / total;
	calib[2] = ((sum[0] + sum[1] + sum[2]) * 256 + total / 2) / total;

	// print calibration information
	printf("calibration command:\n\n");
	printf("\tsetCalibrationData(%d, %d, %d);\n\n", 
		calib[0], calib[1], calib[2]);
}


// set the phase size calibration, use the "substep_calibrate_phases" function
// to get the values. Many encoders (especially low cost ones) have phases that
// don't have the same size. To get good substep accuracy, the code should know
// about this. This is specially important at low speeds with encoders that have
// big phase size differences
void Encoder::setCalibrationData(int step0, int step1, int step2)
{
	this->calibration_data[0] = 0;
	this->calibration_data[1] = step0;
	this->calibration_data[2] = step1;
	this->calibration_data[3] = step2;
}

