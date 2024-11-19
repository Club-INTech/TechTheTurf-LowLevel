#pragma  once

#include "action/dynamixel_xl430.hpp"
#include <asserv/control_loop.hpp>
#include <shared/led_provider.hpp>

#define BLINKER_PERIOD 0.30f
#define CENTER_PERIOD 0.15f

#define CENTER_DIM 20

#define BRAKE_DIM 25
#define BRAKE_RGB 0xFF0000

#define HEADLIGHTS_DIM 30
#define HEADLIGHTS_DIM_RGB 0xFFFF69
#define HEADLIGHTS_RGB 0xA6D3F5

#define RING_BRIGHTNESS 150
#define RING_BRIGHTNESS_DIM 30

#define BLINKER_RGB 0xFF4000

#define INTECH_BLUE 0x005A9F
#define INTECH_LIGHT_BLUE 0x99A7CF
#define INTECH_YELLOW 0xFBCD00

#define REVERSE_LIGHT_BRIGHTNESS 60

#define DISCO_TIME 5

enum class ControlState {
	off = 0,
	automatic,
	manual,
	gay
};

enum class BlinkerState {
	off = 0,
	left,
	right,
	warning,
	estop
};

enum class HeadlightState {
	off = 0,
	dim,
	full
};

enum class RingState {
	off = 0,
	rainbow,
	speed,
	chase,
	wiper,
	police
};

class Effects
{
public:
	Effects(ControlLoop *cl, LedProvider* prov, uint8_t center_brake_pin);
	~Effects();

	void setControlState(ControlState state) {
		this->controlState = state;
		if (state == ControlState::off)
			this->leds->clear();
	}
	void setBlinker(BlinkerState state) {this->blinkers = state;}
	void setRing(RingState state) {this->ringState = state;}
	void setStop(bool en) {this->stopping = en;}
	void setCenterStop(bool en) {this->stopCenter = en;}
	void setDisco(bool dis) {this->ringDisco = dis;}
	void setHeadlights(HeadlightState state) {this->headlights = state;}
	void setReversing(bool rev) {this->reversing = rev;}

	ControlState getControlState() {return this->controlState;}
	BlinkerState getBlinker() {return this->blinkers;}
	HeadlightState getHeadlights() {return this->headlights;}
	RingState getRingState() {return this->ringState;}
	bool getDisco() {return this->ringDisco;}
	bool getStop() {return this->stopping;}
	bool getCenterStop() {return this->stopCenter;}
	bool getReversing() {return this->reversing;}

	// Handle all the stuff
	void work();

	LedProvider *leds;
private:
	ControlLoop *cl;
	uint8_t center_brake_pin;
	absolute_time_t lastTime;

	long firstPixelHue;
	size_t chaseOffset;
	size_t wiperState;

	ControlState controlState;
	BlinkerState blinkers;
	HeadlightState headlights;
	RingState ringState;
	bool stopping;
	bool stopCenter;
	bool reversing;
	bool ringDisco;

	float discoTimer;
	float blinkerTimer;
	float fancyBlinkerTimer;
	float centerTimer;
	float rainbowTimer;
};