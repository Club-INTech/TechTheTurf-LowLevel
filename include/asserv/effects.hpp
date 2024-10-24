#pragma  once

#include <asserv/control_loop.hpp>
#include <shared/neopixel_connect.h>
#include <cstdint>

#define BLINKER_PERIOD 0.25f
#define CENTER_PERIOD 0.15f

#define CENTER_DIM 20
#define BRAKE_DIM 25
#define HEADLIGHT_DIM 30

enum BlinkerState {
	none = 0,
	left,
	right,
	warning,
	estop
};

enum HeadlightState {
	off = 0,
	dim,
	full
};

enum RingState {
	blank = 0,
	rainbow,
	speed
};

class Effects
{
public:
	Effects(ControlLoop *cl, uint8_t left_stop_pin, uint8_t left_blinker_pin, uint8_t right_stop_pin, uint8_t right_blinker_pin, uint8_t center_brake_pin, 
			uint8_t left_headlight, uint8_t right_headlight, uint8_t ws_pin, uint8_t number_leds);
	~Effects();

	void setAuto(bool val) {this->autoMode = val;}
	void setBlinker(BlinkerState state) {this->blinkers = state;}
	void setStop(bool en) {this->stopping = en;}
	void setCenterStop(bool en) {this->stopCenter = en;}
	void setHeadlights(HeadlightState state) {this->headlights = state;}

	bool getAuto() {return this->autoMode;}
	BlinkerState getBlinker() {return this->blinkers;}
	bool getStop() {return this->stopping;}
	bool getCenterStop() {return this->stopCenter;}
	HeadlightState getHeadlights() {return this->headlights;}

	// Handle all the stuff
	void work();

private:
	ControlLoop *cl;
	uint8_t left_stop_pin, right_stop_pin, center_brake_pin, left_blink_pin, right_blink_pin, left_headlight, right_headlight, ws_pin;
	absolute_time_t lastTime;

	NeoPixelConnect pixels;
	long firstPixelHue;

	BlinkerState blinkers;
	HeadlightState headlights;
	bool stopping;
	bool stopCenter;

	bool autoMode;

	float blinkerTimer;
	float centerTimer;
	float ringTimer;
};