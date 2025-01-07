#include <cstring>
#include <shared/led_provider.hpp>
#include <shared/neopixel_connect.h>
#include <algorithm>
#include <hardware/gpio.h>
#include <hardware/clocks.h>
#include <asserv/speed_profile.hpp>
#include <pico/rand.h>
#include <hardware/pwm.h>
#include <asserv/controller.hpp>
#include <asserv/effects.hpp>
#include <cmath>
#include <cstdint>
#include <hardware/timer.h>

Effects::Effects(ControlLoop *cl, LedProvider* prov, Piezo* piezo, uint8_t center_brake_pin) : leds(prov), piezo(piezo), cl(cl), center_brake_pin(center_brake_pin) {
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
	this->ringDisco = false;
	this->stopping = false;
	this->stopCenter = false;
	this->reversing = false;
	this->smoking = false;

	this->leds->setColor(0x0);
	this->leds->display();
	this->firstPixelHue = 0;
	this->chaseOffset = 0;
	this->wiperState = 0;
	this->smokeIdx = 0;
	this->smokeLen = std::strlen(PIEZO_FIRE_STR);

	this->discoTimer = 0;
	this->centerTimer = 0;
	this->fancyBlinkerTimer = 0;
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

	// If we're not controlled, still display
	if (this->controlState == ControlState::off) {
		this->leds->display();
		return;
	}

	// Generate light controls from state when in automatic
	if (this->controlState == ControlState::automatic) {
		Target dl = this->cl->ctrl->getDeltaTarget();

		if (this->cl->running && this->cl->ctrl->isEstopped()) {
			this->blinkers = BlinkerState::estop;
		} else if (this->cl->running && this->cl->ctrl->getState() == ControllerState::reachingTheta) {
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

		bool reversing = dl.dst < 0 && cState == ControllerState::reachingDst;

		this->reversing = this->cl->running && reversing;
		this->stopping = this->cl->running && braking;
		this->headlights = HeadlightState::off;

		this->stopCenter = this->cl->running;
		this->smoking = this->cl->running && cState == ControllerState::reachedTarget;
	}

	// Apply effects from states

	// Ring effects
	float speedDir = std::signbit(this->cl->rCurrentSpeed) ? -1.0f : 1.0f;
	float speed = (std::fabs(this->cl->lCurrentSpeed) + std::fabs(this->cl->rCurrentSpeed))/2.0f;
	float rainbowPeriod = this->ringState == RingState::speed ? std::clamp(1.0f/speed, 4e-3f, 32e-3f) : 16e-3f;

	bool enablePiezo = this->controlState == ControlState::gay ? true : this->smoking;
	this->piezo->setEnable(enablePiezo);
	if (this->controlState != ControlState::gay) {
		this->smokeTimer += dt;
		size_t newIdx = (this->smokeIdx + 1) % this->smokeLen;
		float percentage = this->smokeTimer/PIEZO_FIRE_PERIOD;
		float lightval = float(convertLight(PIEZO_FIRE_STR[this->smokeIdx]))*(1.0f-percentage) + float(convertLight(PIEZO_FIRE_STR[newIdx]))*percentage;
		//uint8_t lightval = convertLight(PIEZO_FIRE_STR[this->smokeIdx]);
		this->leds->setColor(PIEZO_FIRE_LED, enablePiezo ? std::min(int(PIEZO_FIRE_BRIGHT*(1+(lightval-1.0f)/PIEZO_FIRE_DIV)),255) : 0, LedFunction::smokeLight);
		if (this->smokeTimer >= PIEZO_FIRE_PERIOD) {
			this->smokeIdx = newIdx;
			this->smokeTimer = 0;
		}
	}

	if (this->controlState != ControlState::gay && this->ringState == RingState::off) {
		// Turn off the ring
		this->leds->setColor(0x0, 0x0, LedFunction::ringLight);
	} else if ((this->ringState == RingState::rainbow || this->ringState == RingState::speed || this->controlState == ControlState::gay) && this->rainbowTimer >= rainbowPeriod) {
		// Ring rainbow + Gay mode
		if (this->firstPixelHue >= 5*65536)
			this->firstPixelHue = 0;

		LedFunction lfunc = this->controlState == ControlState::gay ? LedFunction::all : LedFunction::ringLight;

		size_t ringSize = this->leds->getSizeParam(lfunc);
		size_t i=0;
		for (size_t idx : this->leds->range(lfunc)) {
			int pixelHue = this->firstPixelHue + (i * 65536L / ringSize);
			this->leds->setColorRaw(idx, NeoPixelConnect::ColorHSV(pixelHue), this->controlState == ControlState::gay ? RING_BRIGHTNESS : RING_BRIGHTNESS_DIM);
			i++;
		}
		this->firstPixelHue += this->ringState == RingState::speed ? 256*speedDir : 256;
		this->rainbowTimer = 0;
	} else if (this->ringState == RingState::chase && this->rainbowTimer >= rainbowPeriod) {
		// Chase mode
		size_t ringSize = this->leds->getSizeParam(LedFunction::ringLight);
		size_t i = 0;
		for (size_t idx : this->leds->range(LedFunction::ringLight)) {
			if (i == this->chaseOffset)
				this->leds->setColorRaw(idx, INTECH_BLUE, RING_BRIGHTNESS);
			else if (i == (this->chaseOffset + ringSize/2) % ringSize)
				this->leds->setColorRaw(idx, INTECH_YELLOW, RING_BRIGHTNESS);
			else
				this->leds->setColorRaw(idx, 0, 0);
			i++;
		}
		this->chaseOffset++;
		this->chaseOffset %= ringSize;
		this->rainbowTimer = 0;
	} else if (this->ringState == RingState::wiper && this->rainbowTimer >= rainbowPeriod) {
		// Wiper mode
		size_t ringSize = this->leds->getSizeParam(LedFunction::ringLight);
		size_t i = 0;
		for (size_t idx : this->leds->range(LedFunction::ringLight)) {
			if (i == this->chaseOffset)
				this->leds->setColorRaw(idx, this->wiperState == 0 ? INTECH_BLUE : INTECH_YELLOW, RING_BRIGHTNESS);
			i++;
		}
		this->chaseOffset++;
		if (this->chaseOffset >= ringSize) {
			this->chaseOffset = 0;
			this->wiperState = (this->wiperState+1)%2;
		}
		this->rainbowTimer = 0;
	} else if (this->ringState == RingState::police) {
		// Police mode
		size_t ringSize = this->leds->getSizeParam(LedFunction::ringLight);
		size_t i = 0;
		for (size_t idx : this->leds->range(LedFunction::ringLight)) {
			if (i >= ringSize/2)
				this->leds->setColorRaw(idx, 0x0000FF, RING_BRIGHTNESS);
			else
				this->leds->setColorRaw(idx, 0xFF0000, RING_BRIGHTNESS);
			i++;
		}
		if (this->rainbowTimer >= rainbowPeriod)
			this->rainbowTimer = 0;
	}
	this->rainbowTimer += dt;

	// Only apply other effects if we're in a "normal" control mode
	if (this->controlState != ControlState::gay) {
		// Fancy blinkers animation...
		float period = (this->blinkers == BlinkerState::estop ? BLINKER_PERIOD/2.0f : BLINKER_PERIOD);
		size_t blinkerSize = this->leds->getSizeParam(LedFunction::fancyBlinker, LedPosition::right);
		float blinkerProgress = std::clamp(this->blinkerTimer / (period*0.5f), 0.0f, 1.0f);
		size_t blinkerCurrentPos = std::min(std::floor(blinkerProgress * ((float)blinkerSize)), (float)blinkerSize-1);
		float maxLedProgress = (1.0f/((float)blinkerSize));
		uint8_t blinkerBrightness = 255*((blinkerProgress - maxLedProgress*blinkerCurrentPos)/maxLedProgress);
		if (this->blinkers != BlinkerState::off && this->blinkers != BlinkerState::left) {
			size_t idx = 0;
			for (size_t pos : this->leds->range(LedFunction::fancyBlinker, LedPosition::right)) {
				this->leds->setColorRaw(pos, BLINKER_RGB, this->blinkerTimer <= period ? idx > blinkerCurrentPos ? 0 : idx == blinkerCurrentPos ? blinkerBrightness : 255 : 0);
				idx++;
			}
		}
		blinkerCurrentPos = blinkerSize-1-blinkerCurrentPos;
		if (this->blinkers != BlinkerState::off && this->blinkers != BlinkerState::right) {
			size_t idx = 0;
			for (size_t pos : this->leds->range(LedFunction::fancyBlinker, LedPosition::left)) {
				this->leds->setColorRaw(pos, BLINKER_RGB, this->blinkerTimer <= period ? idx < blinkerCurrentPos ? 0 : idx == blinkerCurrentPos ? blinkerBrightness : 255 : 0);
				idx++;
			}
		}

		// Normal blinkers
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

		if (!this->reversing)
			this->leds->setColor(0xFFFFFF, 0, LedFunction::reverseLight);

		// Stop lights
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

		this->leds->setColor(this->headlights == HeadlightState::full ? HEADLIGHTS_RGB : HEADLIGHTS_DIM_RGB,
			this->headlights == HeadlightState::off ? 0 : this->headlights == HeadlightState::full ? 255 : HEADLIGHTS_DIM, LedFunction::headlight);

		if (this->ringDisco) {
			if (this->discoTimer >= DISCO_TIME) {
				RingState oldState = this->ringState;
				while (this->ringState == oldState)
					this->ringState = RingState((rand()%4)+1);
				this->discoTimer = 0;
			}
			this->discoTimer += dt;
		}

		if (this->reversing)
			this->leds->setColor(0xFFFFFF, REVERSE_LIGHT_BRIGHTNESS, LedFunction::reverseLight);
	}

	this->leds->display();
}