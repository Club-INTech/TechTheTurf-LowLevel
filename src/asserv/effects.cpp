#include "shared/neopixel_connect.h"
#include <hardware/gpio.h>
#include <hardware/clocks.h>
#include <asserv/speed_profile.hpp>
#include <hardware/pwm.h>
#include <asserv/controller.hpp>
#include <asserv/effects.hpp>
#include <cmath>
#include <cstdint>
#include <hardware/timer.h>

Effects::Effects(ControlLoop *cl, LedProvider* prov, uint8_t center_brake_pin) {
	this->cl = cl;
	this->center_brake_pin = center_brake_pin;
	this->leds = prov;

	gpio_set_function(center_brake_pin, GPIO_FUNC_PWM);

	uint cbslice = pwm_gpio_to_slice_num(center_brake_pin);

	float freq = 40e3;
	uint resolution = 256;
	float div = (float)clock_get_hz(clk_sys) / (freq*resolution);
	pwm_set_wrap(cbslice, resolution-1);
	pwm_set_clkdiv(cbslice, div);
	pwm_set_enabled(cbslice, true);

	pwm_set_chan_level(cbslice, pwm_gpio_to_channel(this->center_brake_pin), 0);
	pwm_set_both_levels(cbslice, 0, 0);

	gpio_set_drive_strength(center_brake_pin, GPIO_DRIVE_STRENGTH_12MA);

	this->lastTime = get_absolute_time();

	this->controlState = ControlState::automatic;
	this->blinkers = BlinkerState::off;
	this->headlights = HeadlightState::off;
	this->ringState = RingState::off;
	this->stopping = false;
	this->stopCenter = false;

	this->leds->setColor(0x0);
	this->leds->display();
	this->firstPixelHue = 0;

	this->centerTimer = 0;
	this->blinkerTimer = 0;
	this->rainbowTimer = 0;
}

Effects::~Effects() {
	gpio_deinit(this->center_brake_pin);
}

void Effects::work() {
	absolute_time_t time = get_absolute_time();
	float dt = ((float)absolute_time_diff_us(this->lastTime, time))/((float)1e6);
	this->lastTime = time;

	// Generate light controls from state
	if (this->controlState == ControlState::automatic) {
		if (this->cl->running && this->cl->ctrl->isEstopped()) {
			this->blinkers = BlinkerState::estop;
		} else if (this->cl->running && this->cl->ctrl->getState() == ControllerState::reachingTheta) {
			Target dl = this->cl->ctrl->getDeltaTarget();
			float dth = dl.theta;
			if (std::abs(dth) >= 0.2f) {
				if (dth > 0)
					this->blinkers = BlinkerState::left;
				else
					this->blinkers = BlinkerState::right;
			}
		} else {
			this->blinkers = BlinkerState::off;
		}

		ControllerState cState = this->cl->ctrl->getState();
		bool braking = ((cState == ControllerState::reachingDst && this->cl->ctrl->spDst->getState() == SpeedProfileState::decelerate) || 
						(cState == ControllerState::reachingTheta && this->cl->ctrl->spAngle->getState() == SpeedProfileState::decelerate) /*|| cState == ControllerState::reachedTarget*/);

		this->stopping = this->cl->running && braking;
		this->headlights = HeadlightState::off;
	}

	if (this->controlState != ControlState::gay) {

		// This is hard coded for now
		this->stopCenter = this->cl->running;

		// Apply state to lights
		uint32_t rgb;
		switch (this->blinkers) {
			case BlinkerState::off:
				this->leds->setColor(0x0, 255, LedFunction::blinker, LedPosition::agnostic);
				this->blinkerTimer = 0;
				break;
			case BlinkerState::right:
			case BlinkerState::left:
			case BlinkerState::warning:
				rgb = this->blinkerTimer <= BLINKER_PERIOD ? BLINKER_RGB : 0x0;
				if (this->blinkers == BlinkerState::warning) {
					this->leds->setColor(rgb, 255, LedFunction::blinker, LedPosition::agnostic);
				} else {
					this->leds->setColor(this->blinkers == BlinkerState::left ? rgb : 0x0, 255, LedFunction::blinker, LedPosition::left);
					this->leds->setColor(this->blinkers == BlinkerState::right ? rgb : 0x0, 255, LedFunction::blinker, LedPosition::right);
				}

				this->blinkerTimer += dt;

				if (this->blinkerTimer >= 2*BLINKER_PERIOD)
					this->blinkerTimer = 0.0f;
				break;
			case BlinkerState::estop:
				rgb = this->blinkerTimer <= BLINKER_PERIOD/2.0f ? BLINKER_RGB : 0x0;
				this->leds->setColor(rgb, 255, LedFunction::blinker, LedPosition::agnostic);

				this->blinkerTimer += dt;

				if (this->blinkerTimer >= BLINKER_PERIOD)
					this->blinkerTimer = 0;
				break;
		}

		if (this->stopping) {
			this->leds->setColor(BRAKE_RGB, 255, LedFunction::brakeLight, LedPosition::rear);
			pwm_set_chan_level(pwm_gpio_to_slice_num(this->center_brake_pin), pwm_gpio_to_channel(this->center_brake_pin), 255);
		} else {
			this->leds->setColor(BRAKE_RGB, this->headlights == HeadlightState::off ? 0 : BRAKE_DIM, LedFunction::brakeLight, LedPosition::rear);

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

		this->leds->setColor(HEADLIGHTS_RGB, this->headlights == HeadlightState::off ? 0 : this->headlights == HeadlightState::full ? 255 : HEADLIGHTS_DIM, LedFunction::headlight);
	}

	if (this->controlState != ControlState::gay && this->ringState == RingState::off) {
		this->leds->setColor(0x0, 0x0, LedFunction::ringLight);
	} else if ((this->ringState == RingState::rainbow || this->controlState == ControlState::gay) && this->rainbowTimer >= 16e-3) {
		if (this->firstPixelHue >= 5*65536)
			this->firstPixelHue = 0;

		LedFunction lfunc = this->controlState == ControlState::gay ? LedFunction::all : LedFunction::ringLight;

		size_t ringSize = this->leds->getSizeParam(lfunc);
		size_t i=0;
		for (LedProvider::Iterator it = this->leds->begin(lfunc); it != this->leds->end(lfunc); ++it) {
			int pixelHue = this->firstPixelHue + (i * 65536L / ringSize);
			this->leds->setColorRaw(*it, NeoPixelConnect::ColorHSV(pixelHue), RING_BRIGHTNESS);
			i++;
		}
		this->firstPixelHue += 256;
		this->rainbowTimer = 0;
	}
	this->rainbowTimer += dt;


	this->leds->display();
}