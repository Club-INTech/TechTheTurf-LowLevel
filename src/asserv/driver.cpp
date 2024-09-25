#include <asserv/driver.hpp>

#include <pico/stdlib.h>
#include <hardware/timer.h>
#include <hardware/clocks.h>
#include <hardware/pwm.h>

// For std::clamp
#include <algorithm>

// BD623x
// PWM must be 20khz-100khz

#define PWM_MODE_A
//#define PWM_MODE_B

#ifdef PWM_MODE_A
#define PWM_OTHER_STATE GPIO_OVERRIDE_LOW
#else
#define PWM_OTHER_STATE GPIO_OVERRIDE_HIGH
#endif

Driver::Driver(uint fin, uint rin, bool reversed, uint resolution, float freq, float dutyOffset) {
	uint fs = pwm_gpio_to_slice_num(fin);
	assert(fs == pwm_gpio_to_slice_num(rin));

	this->reversed = reversed;
	if (fin > rin) {
		this->reversed ^= true;
		fin = rin;
	}

	this->pins = fin;
	this->currentDuty = 0;
	setDutyOffset(dutyOffset);

	gpio_set_function(pins, GPIO_FUNC_PWM);
	gpio_set_function(pins + 1, GPIO_FUNC_PWM);

	this->slice = fs;

	setResolution(resolution);
	setFreq(freq); // sets clkDiv inside

	// Put driver into standby
	gpio_set_outover(this->pins, GPIO_OVERRIDE_LOW);
	gpio_set_outover(this->pins + 1, GPIO_OVERRIDE_LOW);
	pwm_set_enabled(this->slice, false);
	this->running = false;
	this->braking = false;
}

Driver::~Driver() {
	// Disable PWM
	pwm_set_enabled(this->slice, false);
	// Remove all overrides
	gpio_set_outover(this->pins, GPIO_OVERRIDE_NORMAL);
	gpio_set_outover(this->pins + 1, GPIO_OVERRIDE_NORMAL);
	// Reset pins to normal I/O
	gpio_set_function(this->pins, GPIO_FUNC_NULL);
	gpio_set_function(this->pins + 1, GPIO_FUNC_NULL);
}

void Driver::setFreq(float freq) {
	this->frequency = freq;

	// PWM clkdiv is on the increment and not the signal itself
	float div = (float)clock_get_hz(clk_sys) / (freq*this->resolution);
	setClkDiv(div);
}

void Driver::setResolution(uint resolution) {
	this->resolution = resolution;
	pwm_set_wrap(this->slice, resolution-1);

	// Update current level with new resolution
	setRawPwm(this->currentDuty);
}

void Driver::setDutyOffset(float offset) {
	this->dutyOffset = offset;
}

void Driver::setClkDiv(float div) {
	pwm_set_clkdiv(this->slice, div);
}

void Driver::setClkDiv(uint8_t divInt, uint8_t divFrac) {
	pwm_set_clkdiv_int_frac(this->slice, divInt, divFrac);
}

void Driver::setPwm(float duty) {
	// Bail if we're not running
	if (!this->running)
		return;

	// Make sure we're in the the range
	duty = std::clamp(duty,-1.0f,1.0f);

	bool fwd = true;

	if (duty < 0) {
		fwd = false;
		duty = -duty;
	}

	// Account for reversed motor
	fwd ^= this->reversed;

	// Adjust for motor working range
	if (duty != 0.0f)
		duty += this->dutyOffset;

	this->currentDuty = duty;

	setRawPwm(duty);

	// Brake
	if (duty == 0.0f) {
		if (this->braking)
			return;
		gpio_set_outover(this->pins, GPIO_OVERRIDE_HIGH);
		gpio_set_outover(this->pins + 1, GPIO_OVERRIDE_HIGH);
		this->braking = true;
		return;
	} else if (this->braking) {
		gpio_set_outover(this->pins, GPIO_OVERRIDE_NORMAL);
		gpio_set_outover(this->pins + 1, GPIO_OVERRIDE_NORMAL);
		this->braking = false;
	}

	// Otherwise standard control
	if (fwd) {
		gpio_set_outover(this->pins, GPIO_OVERRIDE_NORMAL);
		gpio_set_outover(this->pins + 1, PWM_OTHER_STATE);
	} else {
		gpio_set_outover(this->pins, PWM_OTHER_STATE);
		gpio_set_outover(this->pins + 1, GPIO_OVERRIDE_NORMAL);
	}
}

void Driver::setEnable(bool enabled) {
	if (!this->running && enabled) {
		this->running = true;
		gpio_set_outover(this->pins, GPIO_OVERRIDE_NORMAL);
		gpio_set_outover(this->pins + 1, GPIO_OVERRIDE_NORMAL);
		pwm_set_enabled(this->slice, true);
	} else if (this->running && !enabled) {
		this->running = false;
		gpio_set_outover(this->pins, GPIO_OVERRIDE_LOW);
		gpio_set_outover(this->pins + 1, GPIO_OVERRIDE_LOW);
		pwm_set_enabled(this->slice, false);
	}
}

void Driver::setRawPwm(float duty) {
	// Make sure duty is in the right range
	duty = std::clamp(duty, 0.0f, 1.0f);

	// Calculate level and apply to PWM slice
	int16_t level = (int16_t)(((float)this->resolution) * duty);
	pwm_set_both_levels(this->slice, level, level);
}