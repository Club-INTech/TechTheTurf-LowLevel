#include <hardware/clocks.h>
#include <asserv/speed_profile.hpp>
#include <hardware/pwm.h>
#include <asserv/controller.hpp>
#include <asserv/effects.hpp>
#include <cmath>
#include <cstdint>

Effects::Effects(ControlLoop *cl, uint8_t left_stop_pin, uint8_t left_blinker_pin, uint8_t right_stop_pin, uint8_t right_blinker_pin, uint8_t center_brake_pin) {
	this->cl = cl;
	this->left_stop_pin = left_stop_pin;
	this->right_stop_pin = right_stop_pin;
	this->left_blink_pin = left_blinker_pin;
	this->right_blink_pin = right_blinker_pin;
	this->center_brake_pin = center_brake_pin;

	gpio_init(left_stop_pin);
	gpio_init(right_stop_pin);
	gpio_init(left_blinker_pin);
	gpio_init(right_blinker_pin);
	gpio_set_function(center_brake_pin, GPIO_FUNC_PWM);

	uint slice = pwm_gpio_to_slice_num(center_brake_pin);

	float freq = 40e3;
	uint resolution = 256;
	pwm_set_wrap(slice, resolution-1);
	float div = (float)clock_get_hz(clk_sys) / (freq*resolution);
	pwm_set_clkdiv(slice, div);

	pwm_set_enabled(slice, true);

	pwm_set_chan_level(slice, pwm_gpio_to_channel(this->center_brake_pin), 0);

	gpio_set_dir(left_stop_pin, true);
	gpio_set_dir(right_stop_pin, true);
	gpio_set_dir(left_blinker_pin, true);
	gpio_set_dir(right_blinker_pin, true);

	gpio_set_drive_strength(left_stop_pin, GPIO_DRIVE_STRENGTH_12MA);
	gpio_set_drive_strength(right_stop_pin, GPIO_DRIVE_STRENGTH_12MA);
	gpio_set_drive_strength(left_blinker_pin, GPIO_DRIVE_STRENGTH_12MA);
	gpio_set_drive_strength(right_blinker_pin, GPIO_DRIVE_STRENGTH_12MA);

	gpio_put(left_stop_pin, false);
	gpio_put(right_stop_pin, false);
	gpio_put(left_blinker_pin, false);
	gpio_put(right_blinker_pin, false);

	this->center_timer = 0;
	this->blinker_timer = 0;
}

Effects::~Effects() {
	gpio_deinit(this->left_stop_pin);
	gpio_deinit(this->right_stop_pin);
	gpio_deinit(this->left_blink_pin);
	gpio_deinit(this->right_blink_pin);
}

void Effects::work() {
	float dt = cl->lastDt;

	if (this->cl->running && this->cl->ctrl->isEstopped()) {
		gpio_put(this->left_blink_pin, this->blinker_timer <= BLINKER_PERIOD/2.0f);
		gpio_put(this->right_blink_pin, this->blinker_timer <= BLINKER_PERIOD/2.0f);
		if (this->blinker_timer >= BLINKER_PERIOD)
			this->blinker_timer = 0;

		this->blinker_timer += dt;
	} else if (this->cl->running && this->cl->ctrl->getState() == ControllerState::reachingTheta) {
		Target dl = this->cl->ctrl->getDeltaTarget();
		float dth = dl.theta;
		if (abs(dth) >= 0.2f) {
			if (dth > 0)
				gpio_put(this->left_blink_pin, this->blinker_timer <= BLINKER_PERIOD);
			else
				gpio_put(this->right_blink_pin, this->blinker_timer <= BLINKER_PERIOD);

			this->blinker_timer += dt;

			if (this->blinker_timer >= 2*BLINKER_PERIOD)
				this->blinker_timer = 0.0f;	
		}
	} else {
		gpio_put(this->left_blink_pin, false);
		gpio_put(this->right_blink_pin, false);
		this->blinker_timer = 0;
	}

	ControllerState cState = this->cl->ctrl->getState();
	bool braking = ((cState == ControllerState::reachingDst && this->cl->ctrl->spDst->getState() == SpeedProfileState::decelerate) || 
					(cState == ControllerState::reachingTheta && this->cl->ctrl->spAngle->getState() == SpeedProfileState::decelerate) /*|| cState == ControllerState::reachedTarget*/);

	if (this->cl->running && braking) {
		gpio_put(this->left_stop_pin, true);
		gpio_put(this->right_stop_pin, true);
		pwm_set_chan_level(pwm_gpio_to_slice_num(this->center_brake_pin), pwm_gpio_to_channel(this->center_brake_pin), 255);
	} else {
		gpio_put(this->left_stop_pin, false);
		gpio_put(this->right_stop_pin, false);

		if (this->cl->running)
			pwm_set_chan_level(pwm_gpio_to_slice_num(this->center_brake_pin), pwm_gpio_to_channel(this->center_brake_pin), this->center_timer <= CENTER_PERIOD/2.0f ? 20 : 0);
		else
			pwm_set_chan_level(pwm_gpio_to_slice_num(this->center_brake_pin), pwm_gpio_to_channel(this->center_brake_pin), 0);
	}
	this->center_timer += dt;
	if (this->center_timer >= CENTER_PERIOD)
		this->center_timer = 0;
}