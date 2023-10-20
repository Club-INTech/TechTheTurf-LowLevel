#include <driver.hpp>
#include <pico/stdlib.h>
#include <hardware/timer.h>
#include <hardware/clocks.h>
#include <hardware/pwm.h>

// BD623x
// PWM must be 20khz-100khz

#define PWM_MODE_A
//#define PWM_MODE_B

#ifdef PWM_MODE_A
#define PWM_OTHER_STATE GPIO_OVERRIDE_LOW
#else
#define PWM_OTHER_STATE GPIO_OVERRIDE_HIGH
#endif

Driver::Driver(uint fin_rin, uint resolution, float freq, float dutyOffset) {
	assert((fin_rin % 2) == 0);
	this->pins = fin_rin;
	this->resolution = resolution;
	this->frequency = freq;
	this->currentDuty = 0;
	setDutyOffset(dutyOffset);

	gpio_set_function(pins, GPIO_FUNC_PWM);
	gpio_set_function(pins + 1, GPIO_FUNC_PWM);

	this->slice = pwm_gpio_to_slice_num(fin_rin);

	pwm_set_wrap(this->slice, resolution);

	updateClkdiv();

	gpio_set_outover(this->pins, GPIO_OVERRIDE_LOW);
	gpio_set_outover(this->pins + 1, GPIO_OVERRIDE_LOW);
	pwm_set_enabled(this->slice, false);
	this->running = false;
}

Driver::~Driver() {
	pwm_set_enabled(this->slice, false);
}

void Driver::setFreq(float freq) {
	this->frequency = freq;
	updateClkdiv();
}

void Driver::setResolution(uint resolution) {
	this->resolution = resolution;
	updateClkdiv();
}

void Driver::setDutyOffset(float offset) {
	this->dutyOffset = offset;
}

void Driver::updateClkdiv() {
	// PWM clkdiv is on the increment and not the signal itself
	float div = (float)clock_get_hz(clk_sys) / (this->frequency*this->resolution);
	pwm_set_clkdiv(this->slice, div);
	int16_t level = (int16_t)(((float)this->resolution) * this->currentDuty);
	pwm_set_both_levels(this->slice, level, level);
}

void Driver::setPwm(float duty) {
	if (duty == 0) {
		if (!running)
			return;

		running = false;
		gpio_set_outover(this->pins, GPIO_OVERRIDE_LOW);
		gpio_set_outover(this->pins + 1, GPIO_OVERRIDE_LOW);
		pwm_set_enabled(this->slice, false);
		return;
	} else if (!running) {
		running = true;
		gpio_set_outover(this->pins, GPIO_OVERRIDE_NORMAL);
		gpio_set_outover(this->pins + 1, GPIO_OVERRIDE_NORMAL);
		pwm_set_enabled(this->slice, true);
	}

	bool fwd = true;

	if (duty < 0) {
		fwd = false;
		duty = -duty;
	}

	// Adjust for motor working range
	duty += this->dutyOffset;

	this->currentDuty = duty;

	int16_t level = (int16_t)(((float)this->resolution) * duty);
	pwm_set_both_levels(this->slice, level, level);

	if (fwd) {
		gpio_set_outover(this->pins, GPIO_OVERRIDE_NORMAL);
		gpio_set_outover(this->pins + 1, PWM_OTHER_STATE);
	} else {
		gpio_set_outover(this->pins, PWM_OTHER_STATE);
		gpio_set_outover(this->pins + 1, GPIO_OVERRIDE_NORMAL);
	}
}