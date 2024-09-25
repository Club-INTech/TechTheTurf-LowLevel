#include "asserv/speed_profile.hpp"
#include <asserv/controller.hpp>
#include <asserv/effects.hpp>
#include <cmath>


Effects::Effects(ControlLoop *cl, uint8_t left_stop_pin, uint8_t left_blinker_pin, uint8_t right_stop_pin, uint8_t right_blinker_pin) {
	this->cl = cl;
	this->left_stop_pin = left_stop_pin;
	this->right_stop_pin = right_stop_pin;
	this->left_blink_pin = left_blinker_pin;
	this->right_blink_pin = right_blinker_pin;

	gpio_init(left_stop_pin);
	gpio_init(right_stop_pin);
	gpio_init(left_blinker_pin);
	gpio_init(right_blinker_pin);

	gpio_set_dir(left_stop_pin, true);
	gpio_set_dir(right_stop_pin, true);
	gpio_set_dir(left_blinker_pin, true);
	gpio_set_dir(right_blinker_pin, true);

	gpio_put(left_stop_pin, false);
	gpio_put(right_stop_pin, false);
	gpio_put(left_blinker_pin, false);
	gpio_put(right_blinker_pin, false);

	this->oldDst = 0.0f;
	this->oldTheta = 0.0f;
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

	float theta = this->cl->odo->theta;
	float dst = this->cl->odo->dst;

	float dTheta = (theta-this->oldTheta)/dt;
	float dDst = (dst-this->oldDst)/dt;

	if (this->cl->running && (this->cl->ctrl->spDst->getState() == SpeedProfileState::decelerate || this->cl->ctrl->getState() == ControllerState::reachedTarget)) {
		gpio_put(this->left_stop_pin, true);
		gpio_put(this->right_stop_pin, true);
	} else {
		gpio_put(this->left_stop_pin, false);
		gpio_put(this->right_stop_pin, false);
	}

	this->oldTheta = theta;
	this->oldDst = dst;
}