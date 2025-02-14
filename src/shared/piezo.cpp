#include <shared/piezo.hpp>

#include <hardware/gpio.h>
#include <hardware/pwm.h>
#include <hardware/clocks.h>

Piezo::Piezo(uint pin, float frequency) : pin(pin) {
	this->slice = pwm_gpio_to_slice_num(pin);
	this->chan = pwm_gpio_to_channel(pin);

	pwm_set_wrap(this->slice, 7);
	setEnable(false);
	setFreq(frequency);

	pwm_set_enabled(this->slice, true);
	gpio_set_function(pin, GPIO_FUNC_PWM);
}

void Piezo::setFreq(float freq) {
	float div = (float)clock_get_hz(clk_sys) / (freq*8);
	pwm_set_clkdiv(this->slice, div);
}

void Piezo::setEnable(bool enable) {
	this->enable = enable;
	pwm_set_chan_level(this->slice, this->chan, this->enable ? 4 : 0);	
}

Piezo::~Piezo() {
	gpio_set_function(pin, GPIO_FUNC_NULL);
	setEnable(false);
}