#include "shared/utils.hpp"
#include <cstring>
#include <shared/led_provider.hpp>
#include <shared/neopixel_connect.h>
#include <algorithm>
#include <hardware/gpio.h>
#include <hardware/clocks.h>
#include <asserv/speed_profile.hpp>
#include <pico/rand.h>
#include <asserv/controller.hpp>
#include <asserv/effects.hpp>
#include <cmath>
#include <cstdint>
#include <hardware/timer.h>

Effects::Effects(ControlLoop *cl, LedProvider* prov, Piezo* piezo, Spoiler *spoiler, PopUp *popup)
 : leds(prov), piezo(piezo), spoiler(spoiler), popup(popup), cl(cl)  {
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
	this->ringTimer = 0;
	this->startupTimer = 0;
}

Effects::~Effects() {
}

void Effects::work() {
	absolute_time_t time = get_absolute_time();
	float dt = ((float)absolute_time_diff_us(this->lastTime, time))/((float)1e6);
	this->lastTime = time;

	if (this->startupTimer <= STARTUP_TIME+BATTERY_TIME) {
		if (this->startupTimer == 0.0f) {
			this->popup->setOpen(true);
			this->leds->setColor(INTECH_YELLOW, RING_BRIGHTNESS, LedFunction::all ^ LedFunction::ringLight, LedPosition::right);
			this->leds->setColor(INTECH_BLUE, RING_BRIGHTNESS, LedFunction::all ^ LedFunction::ringLight, LedPosition::left);
			this->leds->setColor(0xFF0000, RING_BRIGHTNESS, LedFunction::all, LedPosition::rear | LedPosition::center, true);
		}
		if (this->startupTimer <= STARTUP_TIME) {
			uint32_t ringSize = this->leds->getSizeParam(LedFunction::ringLight);
			if (this->ringTimer >= STARTUP_TIME/((ringSize+2)*2.0f)) {
				uint32_t i = 0;
				for (uint32_t idx : this->leds->range(LedFunction::ringLight)) {
					if (i == this->chaseOffset)
						this->leds->setColorRaw(idx, i > ringSize/2 ? INTECH_BLUE : INTECH_YELLOW, this->wiperState == 0 ? RING_BRIGHTNESS : 0);
					i++;
				}

				if (this->wiperState == 0)
					this->chaseOffset++;
				else
					this->chaseOffset--;

				if (this->chaseOffset >= ringSize) {
					this->wiperState = (this->wiperState+1)%2;
				}
				this->ringTimer = 0;
			}
			this->ringTimer += dt;
		} else {
			uint32_t ringSize = this->leds->getSizeParam(LedFunction::ringLight, LedPosition::front, true);
			float batt = std::clamp(calculateLipoPercentage(this->cl->lastPower.voltage)/100.0f, 0.0f, 1.0f);
			float maxSizeflt = ringSize*batt;
			uint32_t maxSize = maxSizeflt;
			maxSizeflt -= maxSize;
			uint32_t i = 0;
			for (uint32_t idx : this->leds->range(LedFunction::ringLight, LedPosition::front, true)) {
				if (i == maxSize)
					this->leds->setColorRaw(idx, 0x00FF00, std::lerp(RING_BATT_BRIGHTNESS_DIM, RING_BRIGHTNESS, maxSizeflt));
				else if (i < maxSize)
					this->leds->setColorRaw(idx, 0x00FF00, RING_BRIGHTNESS);
				else
					this->leds->setColorRaw(idx, 0xFF0000, 10);
				i++;
			}
		}
		this->startupTimer += dt;
		this->leds->display();
		return;
	}

	// If we're not controlled, still display
	if (this->controlState == ControlState::off) {
		if (this->leds != nullptr)
			this->leds->display();
		return;
	}

	// Generate effects controls from state when in automatic
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

		float pop = (this->headlights != HeadlightState::off || this->controlState == ControlState::gay) ? 1 : 0;
		this->leftPop = pop;
		this->rightPop = pop;
	}

	// Apply effects from states

	// Pop up Headlights
	if (this->popup != nullptr) {
		this->popup->setPop(this->leftPop, this->rightPop);
	}

	// Spoiler
	if (this->spoiler != nullptr) {
		// todo
	}

	// Piezo
	if (this->piezo != nullptr) {
		bool enablePiezo = this->controlState == ControlState::gay ? true : this->smoking;
		this->piezo->setEnable(enablePiezo);

		if (this->leds != nullptr && this->controlState != ControlState::gay) {
			this->smokeTimer += dt;
			uint32_t newIdx = (this->smokeIdx + 1) % this->smokeLen;
			float percentage = this->smokeTimer/PIEZO_FIRE_PERIOD;
			float lightval = float(convertLight(PIEZO_FIRE_STR[this->smokeIdx]))*(1.0f-percentage) + float(convertLight(PIEZO_FIRE_STR[newIdx]))*percentage;
			//uint8_t lightval = convertLight(PIEZO_FIRE_STR[this->smokeIdx]);
			this->leds->setColor(PIEZO_FIRE_LED, enablePiezo ? std::min(int(PIEZO_FIRE_BRIGHT*(1+(lightval-1.0f)/PIEZO_FIRE_DIV)),255) : 0, LedFunction::smokeLight);
			if (this->smokeTimer >= PIEZO_FIRE_PERIOD) {
				this->smokeIdx = newIdx;
				this->smokeTimer = 0;
			}
		}
	}

	// LEDs so we bail if we don't have any
	if (this->leds == nullptr)
		return;

	// Ring effects
	if (this->leds->getSizeParam(LedFunction::ringLight) > 0) {
		float speedDir = std::signbit(this->cl->rCurrentSpeed) ? -1.0f : 1.0f;
		float speed = (std::fabs(this->cl->lCurrentSpeed) + std::fabs(this->cl->rCurrentSpeed))/2.0f;
		float rainbowPeriod = this->ringState == RingState::speed ? std::clamp(1.0f/speed, 4e-3f, 32e-3f) : 16e-3f;

		if (this->controlState != ControlState::gay && this->ringState == RingState::off) {
			// Turn off the ring
			this->leds->setColor(0x0, 0x0, LedFunction::ringLight);
		} else if (this->ringState == RingState::rainbow || this->ringState == RingState::speed || this->controlState == ControlState::gay) {
			if (this->ringTimer >= rainbowPeriod) {
				// Ring rainbow + Gay mode
				if (this->firstPixelHue >= 5*65536)
					this->firstPixelHue = 0;

				LedFunction lfunc = this->controlState == ControlState::gay ? LedFunction::all : LedFunction::ringLight;

				uint32_t ringSize = this->leds->getSizeParam(lfunc);
				uint32_t i=0;
				for (uint32_t idx : this->leds->range(lfunc)) {
					int pixelHue = this->firstPixelHue + (i * 65536L / ringSize);
					this->leds->setColorRaw(idx, NeoPixelConnect::ColorHSV(pixelHue), this->controlState == ControlState::gay ? RING_BRIGHTNESS : RING_BRIGHTNESS_DIM);
					i++;
				}
				this->firstPixelHue += this->ringState == RingState::speed ? 256*speedDir : 256;
				this->ringTimer = 0;
			}
		} else if (this->ringState == RingState::chase && this->ringTimer >= rainbowPeriod) {
			// Chase mode
			uint32_t ringSize = this->leds->getSizeParam(LedFunction::ringLight);
			uint32_t i = 0;
			for (uint32_t idx : this->leds->range(LedFunction::ringLight)) {
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
			this->ringTimer = 0;
		} else if (this->ringState == RingState::wiper && this->ringTimer >= rainbowPeriod) {
			// Wiper mode
			uint32_t ringSize = this->leds->getSizeParam(LedFunction::ringLight);
			uint32_t i = 0;
			for (uint32_t idx : this->leds->range(LedFunction::ringLight)) {
				if (i == this->chaseOffset)
					this->leds->setColorRaw(idx, this->wiperState == 0 ? INTECH_BLUE : INTECH_YELLOW, RING_BRIGHTNESS);
				i++;
			}
			this->chaseOffset++;
			if (this->chaseOffset >= ringSize) {
				this->chaseOffset = 0;
				this->wiperState = (this->wiperState+1)%2;
			}
			this->ringTimer = 0;
		} else if (this->ringState == RingState::police && this->ringTimer >= rainbowPeriod) {
			// Police mode
			uint32_t ringSize = this->leds->getSizeParam(LedFunction::ringLight);
			uint32_t i = 0;
			for (uint32_t idx : this->leds->range(LedFunction::ringLight)) {
				if (i >= ringSize/2)
					this->leds->setColorRaw(idx, 0x0000FF, RING_BRIGHTNESS);
				else
					this->leds->setColorRaw(idx, 0xFF0000, RING_BRIGHTNESS);
				i++;
			}
			if (this->ringTimer >= rainbowPeriod)
				this->ringTimer = 0;
		} else if (this->ringState == RingState::battery) {
			// Battery mode
			uint32_t ringSize = this->leds->getSizeParam(LedFunction::ringLight);
			float curr = std::clamp((this->cl->lastPower.current-0.040f)/0.5f, 0.0f, 1.0f); 
			float batt = std::clamp(calculateLipoPercentage(this->cl->lastPower.voltage)/100.0f, 0.0f, 1.0f);
			uint32_t color = colorLerp(colorLerp(0x00FF00, 0xFFFF00, curr), colorLerp(0xFFFF00, 0xFF0000, curr), curr);
			float maxSizeflt = ringSize*batt;
			uint32_t maxSize = maxSizeflt;
			maxSizeflt -= maxSize;
			uint32_t i = 0;
			for (uint32_t idx : this->leds->range(LedFunction::ringLight)) {
				if (i == maxSize)
					this->leds->setColorRaw(idx, color, std::lerp(RING_BATT_BRIGHTNESS_DIM, RING_BRIGHTNESS, maxSizeflt));
				else if (i < maxSize)
					this->leds->setColorRaw(idx, color, RING_BRIGHTNESS);
				else
					this->leds->setColorRaw(idx, color, RING_BATT_BRIGHTNESS_DIM);
				i++;
			}
		}
		this->ringTimer += dt;
	}

	// Only apply other effects if we're in a "normal" control mode
	if (this->controlState != ControlState::gay) {
		// Fancy blinkers animation...
		if (this->leds->getSizeParam(LedFunction::fancyBlinker) > 0) {
			float period = (this->blinkers == BlinkerState::estop ? BLINKER_PERIOD/2.0f : BLINKER_PERIOD);
			uint32_t blinkerSize = this->leds->getSizeParam(LedFunction::fancyBlinker, LedPosition::right);
			float blinkerProgress = std::clamp(this->blinkerTimer / (period*0.5f), 0.0f, 1.0f);
			uint32_t blinkerCurrentPos = std::min(std::floor(blinkerProgress * ((float)blinkerSize)), (float)blinkerSize-1);
			float maxLedProgress = (1.0f/((float)blinkerSize));
			uint8_t blinkerBrightness = 255*((blinkerProgress - maxLedProgress*blinkerCurrentPos)/maxLedProgress);
			if (this->blinkers != BlinkerState::off && this->blinkers != BlinkerState::left) {
				uint32_t idx = 0;
				for (uint32_t pos : this->leds->range(LedFunction::fancyBlinker, LedPosition::right)) {
					this->leds->setColorRaw(pos, BLINKER_RGB, this->blinkerTimer <= period ? idx > blinkerCurrentPos ? 0 : idx == blinkerCurrentPos ? blinkerBrightness : 255 : 0);
					idx++;
				}
			}
			blinkerCurrentPos = blinkerSize-1-blinkerCurrentPos;
			if (this->blinkers != BlinkerState::off && this->blinkers != BlinkerState::right) {
				uint32_t idx = 0;
				for (uint32_t pos : this->leds->range(LedFunction::fancyBlinker, LedPosition::left)) {
					this->leds->setColorRaw(pos, BLINKER_RGB, this->blinkerTimer <= period ? idx < blinkerCurrentPos ? 0 : idx == blinkerCurrentPos ? blinkerBrightness : 255 : 0);
					idx++;
				}
			}
		}

		// Normal blinkers
		if (this->leds->getSizeParam(LedFunction::blinker) > 0) {
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
		}

		if (!this->reversing && this->leds->getSizeParam(LedFunction::reverseLight) > 0)
			this->leds->setColor(0xFFFFFF, 0, LedFunction::reverseLight);

		// Stop lights
		if (this->leds->getSizeParam(LedFunction::brakeLight) > 0) {
			if (this->stopping) {
				this->leds->setColor(BRAKE_RGB, 255, LedFunction::brakeLight, LedPosition::rear);
				//pwm_set_chan_level(pwm_gpio_to_slice_num(this->center_brake_pin), pwm_gpio_to_channel(this->center_brake_pin), 255);
			} else {
				this->leds->setColor(BRAKE_RGB, this->headlights == HeadlightState::off ? 0 : BRAKE_DIM, LedFunction::brakeLight, LedPosition::rear);

				if (this->stopCenter) {
					this->leds->setColor(BRAKE_RGB, this->centerTimer <= CENTER_PERIOD/2.0f ? CENTER_DIM : 0, LedFunction::brakeLight, LedPosition::rear | LedPosition::center, true);
					//pwm_set_chan_level(pwm_gpio_to_slice_num(this->center_brake_pin), pwm_gpio_to_channel(this->center_brake_pin), this->centerTimer <= CENTER_PERIOD/2.0f ? CENTER_DIM : 0);
					this->centerTimer += dt;
					if (this->centerTimer >= CENTER_PERIOD)
						this->centerTimer = 0;
				} else {
					this->leds->setColor(BRAKE_RGB, 0, LedFunction::brakeLight, LedPosition::rear | LedPosition::center, true);
					//pwm_set_chan_level(pwm_gpio_to_slice_num(this->center_brake_pin), pwm_gpio_to_channel(this->center_brake_pin), 0);
					this->centerTimer = 0;
				}
			}
		}

		if (this->leds->getSizeParam(LedFunction::headlight) > 0)
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

		if (this->reversing && this->leds->getSizeParam(LedFunction::reverseLight) > 0)
			this->leds->setColor(0xFFFFFF, REVERSE_LIGHT_BRIGHTNESS, LedFunction::reverseLight);
	}

	this->leds->display();
}