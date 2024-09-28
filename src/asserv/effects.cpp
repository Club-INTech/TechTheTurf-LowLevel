#include <hardware/gpio.h>
#include <hardware/clocks.h>
#include <asserv/speed_profile.hpp>
#include <hardware/pwm.h>
#include <asserv/controller.hpp>
#include <asserv/effects.hpp>
#include <cmath>
#include <cstdint>

Effects::Effects(ControlLoop *cl, uint8_t left_stop_pin, uint8_t left_blinker_pin, uint8_t right_stop_pin, uint8_t right_blinker_pin, uint8_t center_brake_pin,
				uint8_t left_headlight, uint8_t right_headlight) {
	this->cl = cl;
	this->left_stop_pin = left_stop_pin;
	this->right_stop_pin = right_stop_pin;
	this->left_blink_pin = left_blinker_pin;
	this->right_blink_pin = right_blinker_pin;
	this->center_brake_pin = center_brake_pin;
	this->left_headlight = left_headlight;
	this->right_headlight = right_headlight;

	gpio_init(left_blinker_pin);
	gpio_init(right_blinker_pin);
	gpio_set_function(left_stop_pin, GPIO_FUNC_PWM);
	gpio_set_function(right_stop_pin, GPIO_FUNC_PWM);
	gpio_set_function(left_headlight, GPIO_FUNC_PWM);
	gpio_set_function(right_headlight, GPIO_FUNC_PWM);
	gpio_set_function(center_brake_pin, GPIO_FUNC_PWM);

	uint cbslice = pwm_gpio_to_slice_num(center_brake_pin);
	uint sslice = pwm_gpio_to_slice_num(left_stop_pin);
	uint hslice = pwm_gpio_to_slice_num(left_headlight);


	float freq = 40e3;
	uint resolution = 256;
	float div = (float)clock_get_hz(clk_sys) / (freq*resolution);
	pwm_set_wrap(cbslice, resolution-1);
	pwm_set_wrap(sslice, resolution-1);
	pwm_set_wrap(hslice, resolution-1);
	pwm_set_clkdiv(cbslice, div);
	pwm_set_clkdiv(sslice, div);
	pwm_set_clkdiv(hslice, div);

	pwm_set_enabled(cbslice, true);
	pwm_set_enabled(sslice, true);
	pwm_set_enabled(hslice, true);

	pwm_set_chan_level(cbslice, pwm_gpio_to_channel(this->center_brake_pin), 0);
	pwm_set_both_levels(sslice, 0, 0);
	pwm_set_both_levels(cbslice, 0, 0);

	gpio_set_dir(left_blinker_pin, true);
	gpio_set_dir(right_blinker_pin, true);

	gpio_set_drive_strength(left_stop_pin, GPIO_DRIVE_STRENGTH_12MA);
	gpio_set_drive_strength(right_stop_pin, GPIO_DRIVE_STRENGTH_12MA);
	gpio_set_drive_strength(left_blinker_pin, GPIO_DRIVE_STRENGTH_12MA);
	gpio_set_drive_strength(right_blinker_pin, GPIO_DRIVE_STRENGTH_12MA);
	gpio_set_drive_strength(left_headlight, GPIO_DRIVE_STRENGTH_12MA);
	gpio_set_drive_strength(right_headlight, GPIO_DRIVE_STRENGTH_12MA);
	gpio_set_drive_strength(center_brake_pin, GPIO_DRIVE_STRENGTH_12MA);

	gpio_put(left_blinker_pin, false);
	gpio_put(right_blinker_pin, false);

	this->lastTime = get_absolute_time();

	this->blinkers = BlinkerState::none;
	this->headlights = HeadlightState::off;
	this->stopping = false;
	this->stopCenter = false;
	this->autoMode = true;

	this->centerTimer = 0;
	this->blinkerTimer = 0;
}

Effects::~Effects() {
	gpio_deinit(this->left_stop_pin);
	gpio_deinit(this->right_stop_pin);
	gpio_deinit(this->left_blink_pin);
	gpio_deinit(this->right_blink_pin);
	gpio_deinit(this->center_brake_pin);
	gpio_deinit(this->left_headlight);
	gpio_deinit(this->right_headlight);
}

void Effects::work() {
	absolute_time_t time = get_absolute_time();
	float dt = ((float)absolute_time_diff_us(this->lastTime, time))/((float)1e6);
	this->lastTime = time;

	// Generate light controls from state
	if (this->autoMode) {
		if (this->cl->running && this->cl->ctrl->isEstopped()) {
			this->blinkers = BlinkerState::estop;
		} else if (this->cl->running && this->cl->ctrl->getState() == ControllerState::reachingTheta) {
			Target dl = this->cl->ctrl->getDeltaTarget();
			float dth = dl.theta;
			if (abs(dth) >= 0.2f) {
				if (dth > 0)
					this->blinkers = BlinkerState::left;
				else
					this->blinkers = BlinkerState::right;
			}
		} else {
			this->blinkers = BlinkerState::none;
		}

		ControllerState cState = this->cl->ctrl->getState();
		bool braking = ((cState == ControllerState::reachingDst && this->cl->ctrl->spDst->getState() == SpeedProfileState::decelerate) || 
						(cState == ControllerState::reachingTheta && this->cl->ctrl->spAngle->getState() == SpeedProfileState::decelerate) /*|| cState == ControllerState::reachedTarget*/);

		this->stopping = this->cl->running && braking;
		this->headlights = HeadlightState::off;
	}

	// This is hard coded for now
	this->stopCenter = this->cl->running;

	// Apply state to lights
	bool actv;
	switch (this->blinkers) {
		case BlinkerState::none:
			gpio_put(this->left_blink_pin, false);
			gpio_put(this->right_blink_pin, false);
			this->blinkerTimer = 0;
			break;
		case BlinkerState::right:
		case BlinkerState::left:
		case BlinkerState::warning:
			actv = this->blinkerTimer <= BLINKER_PERIOD;
			if (this->blinkers == BlinkerState::warning) {
				gpio_put(this->left_blink_pin, actv);
				gpio_put(this->right_blink_pin, actv);
			} else {
				gpio_put(this->left_blink_pin, this->blinkers == BlinkerState::left ? actv : false);
				gpio_put(this->right_blink_pin, this->blinkers == BlinkerState::right ? actv : false);
			}

			this->blinkerTimer += dt;

			if (this->blinkerTimer >= 2*BLINKER_PERIOD)
				this->blinkerTimer = 0.0f;				
			break;
		case BlinkerState::estop:
			actv = this->blinkerTimer <= BLINKER_PERIOD/2.0f;
			gpio_put(this->left_blink_pin, actv);
			gpio_put(this->right_blink_pin, actv);

			this->blinkerTimer += dt;

			if (this->blinkerTimer >= BLINKER_PERIOD)
				this->blinkerTimer = 0;
			break;
	}

	if (this->stopping) {
		pwm_set_both_levels(pwm_gpio_to_slice_num(this->left_stop_pin), 255, 255);
		pwm_set_chan_level(pwm_gpio_to_slice_num(this->center_brake_pin), pwm_gpio_to_channel(this->center_brake_pin), 255);
	} else {
		if (this->headlights)
			pwm_set_both_levels(pwm_gpio_to_slice_num(this->left_stop_pin), BRAKE_DIM, BRAKE_DIM);
		else
			pwm_set_both_levels(pwm_gpio_to_slice_num(this->left_stop_pin), 0, 0);

		if (this->stopCenter) {
			pwm_set_chan_level(pwm_gpio_to_slice_num(this->center_brake_pin), pwm_gpio_to_channel(this->center_brake_pin), this->centerTimer <= CENTER_PERIOD/2.0f ? CENTER_DIM : 0);
			this->centerTimer += dt;
			if (this->centerTimer >= CENTER_PERIOD)
				this->centerTimer = 0;
		} else {
			pwm_set_chan_level(pwm_gpio_to_slice_num(this->center_brake_pin), pwm_gpio_to_channel(this->center_brake_pin), 0);
			this->centerTimer = 0;
		}
	}

	if (this->headlights != HeadlightState::off) {
		unsigned int val = this->headlights == HeadlightState::full ? 255 : HEADLIGHT_DIM;
		pwm_set_both_levels(pwm_gpio_to_slice_num(this->left_headlight), val, val);
	} else {
		pwm_set_both_levels(pwm_gpio_to_slice_num(this->left_headlight), 0, 0);
	}
}