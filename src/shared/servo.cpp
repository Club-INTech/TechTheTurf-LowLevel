#include <shared/servo.hpp>

#include <hardware/gpio.h>
#include <hardware/pwm.h>
#include <hardware/clocks.h>

#include <cstdint>
#include <cmath>
#include <algorithm>

Servo::Servo(uint pin, double freq, bool centered, double min, double max) : frequency(freq), minTime(min), maxTime(max), centered(centered), pin(pin) {
	this->slice = pwm_gpio_to_slice_num(pin);
	this->chan = pwm_gpio_to_channel(pin);

	updatePWM();
	setRaw(0);
	pwm_set_enabled(this->slice, true);
	gpio_set_function(pin, GPIO_FUNC_PWM);
}

Servo::~Servo() {
	gpio_set_function(pin, GPIO_FUNC_NULL);
	setRaw(0);
}

void Servo::setFreq(double freq) {
	this->frequency = freq;
	updatePWM();
}

void Servo::setCentered(bool centered) {
	this->centered = centered;
}

void Servo::setRange(double min, double max) {
	this->minTime = min;
	this->maxTime = max;
	updatePWM();
}

void Servo::setRaw(double us) {
	uint16_t pwm_val = std::round((us*1e6)/this->realPeriod);
	pwm_set_chan_level(this->slice, this->chan, pwm_val);
}

void Servo::setValue(float val) {
	if (centered)
		val = (val+1.0f)/2.0f;
	val = std::clamp(val, 0.0f, 1.0f);
	uint16_t pwm_val = this->minValue + std::round(float(this->maxValue-this->minValue)*val);
	pwm_set_chan_level(this->slice, this->chan, pwm_val);
}

void Servo::disable() {
	pwm_set_chan_level(this->slice, this->chan, 0);	
}

void Servo::updatePWM() {
	double fsys = double(clock_get_hz(clk_sys));
	uint16_t maxTop = 0xFFFF;

	uint8_t div = 0;
	uint8_t frac = 0;
	uint16_t top = 0;
	bool found = false;
	for (uint16_t d=1;d<256 && !found;d++) {
		for (uint8_t f=0;f<16;f++) {
			double period = (d+(f/16.0))/fsys;
			double minVal = this->minTime/period;
			double maxVal = this->maxTime/period;
			if (maxVal > maxTop)
				continue;
			if (uint(minVal) != minVal || uint(maxVal) != maxVal)
				continue;
			double minFreq = 1.0/(period*maxTop);
			double maxFreq = 1.0/(period*(maxVal+1));
			if (minFreq > this->frequency || maxFreq < this->frequency)
				continue;
			double newTop = (1.0/this->frequency)/period;
			if (int(newTop) != newTop)
				continue;
			div = d;
			frac = f;
			top = uint16_t(newTop);
			this->minValue = uint16_t(minVal);
			this->maxValue = uint16_t(maxVal);
			this->realPeriod = (div+(frac/16.0))/fsys;
			found = true;
			break;
		}
	}

	// "Optimal" method failed, fallback to non accurate values
	if (div == 0) {
		// Calculate divider for the required frequency at 14bits and get other values from that
		top = (1<<14)-1;
		double divider = fsys/(this->frequency*(top+1));

		div = uint8_t(divider);
		frac = std::round((divider - div) * (0x01 << 4));

		this->realPeriod = (div+(frac/16.0))/fsys;
		this->minValue = std::round(this->minTime/this->realPeriod);
		this->maxValue = std::round(this->maxTime/this->realPeriod);
	}

	pwm_set_clkdiv_int_frac(this->slice, div, frac);
	pwm_set_wrap(this->slice, top);
}