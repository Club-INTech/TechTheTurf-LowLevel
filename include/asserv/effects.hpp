#pragma  once

#include <shared/ldr.hpp>
#include <shared/popup.hpp>
#include <shared/spoiler.hpp>
#include <asserv/control_loop.hpp>
#include <shared/led_provider.hpp>
#include <shared/piezo.hpp>

#define BLINKER_PERIOD 0.30f
#define CENTER_PERIOD 0.15f

#define CENTER_DIM 20

#define BRAKE_DIM 25
#define BRAKE_RGB 0xFF0000

#define HEADLIGHTS_DIM 30
#define HEADLIGHTS_DIM_RGB 0xFFFF69
#define HEADLIGHTS_RGB 0xA6D3F5
#define RING_BATT_BRIGHTNESS_DIM 3

#define RING_BRIGHTNESS 150
#define RING_BRIGHTNESS_DIM 30

#define BLINKER_RGB 0xFF4000

#define INTECH_BLUE 0x005A9F
#define INTECH_LIGHT_BLUE 0x99A7CF
#define INTECH_YELLOW 0xFBCD00

#define PIEZO_FIRE_LED 0xFF2000
#define PIEZO_FIRE_STR "fuckminetlesgrosconnards"
#define PIEZO_FIRE_PERIOD 0.05f
#define PIEZO_FIRE_DIV 4.0f
#define PIEZO_FIRE_BRIGHT 150

#define REVERSE_LIGHT_BRIGHTNESS 60

#define STARTUP_EFFECT
#define STARTUP_TIME 1.6f
#define BATTERY_TIME 1.50f

#define POLICE_SLOW_PERIOD 0.18f
#define POLICE_FAST_PERIOD 0.08f
#define POLICE_SLOW_COUNT 8
#define POLICE_FAST_COUNT 12

#define DAYLIGHT_BRIGHT 65

#define DISCO_TIME 5

#define LIGHT_WAIT_TIME 1.5
#define LIGHT_BLIND_WAIT_TIME 0.2
#define LUX_BRIGHTS_LEVEL 3
#define LUX_CRUISE_LEVEL 10
#define LUX_BLINDING_LEVEL 30

enum class ControlState {
	off = 0,
	automatic,
	manual,
	gay,
	police,
	show
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
	battery
};

static inline float convertLight(char val) {
	return ((val - 'a') * 21.25f)/255.0f;
}

class Effects
{
public:
	Effects(ControlLoop *cl, LedProvider* prov, Piezo* piezo, Spoiler *spoiler, PopUp *popup, LDR *ldrExt, LDR *ldrFront);
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
	void setSmoking(bool smoking) {this->smoking = smoking;}
	void setPop(float left, float right) {this->leftPop = left; this->rightPop = right;}

	ControlState getControlState() {return this->controlState;}
	BlinkerState getBlinker() {return this->blinkers;}
	HeadlightState getHeadlights() {return this->headlights;}
	RingState getRingState() {return this->ringState;}
	bool getDisco() {return this->ringDisco;}
	bool getStop() {return this->stopping;}
	bool getCenterStop() {return this->stopCenter;}
	bool getReversing() {return this->reversing;}
	bool getSmoking() {return this->smoking;}
	float getPopLeft() {return this->leftPop;}
	float getPopRight() {return this->rightPop;}

	// Handle all the stuff
	void work();

	LedProvider *leds;
	Piezo *piezo;
	Spoiler *spoiler;
	PopUp *popup;
	LDR *ldrExt, *ldrFront;
private:
	ControlLoop *cl;
	absolute_time_t lastTime;

	long firstPixelHue;
	size_t chaseOffset;
	size_t wiperState;
	size_t smokeIdx;
	size_t smokeLen;
	bool policeSide;
	bool policeFast;

	ControlState controlState;
	BlinkerState blinkers;
	HeadlightState headlights, nextHeadlights;
	RingState ringState;
	bool stopping;
	bool stopCenter;
	bool reversing;
	bool ringDisco;
	bool smoking;
	float leftPop, rightPop;

	float showTimer;
	float headlightsTimer;
	float headlightsBlindTimer;
	float discoTimer;
	float blinkerTimer;
	float fancyBlinkerTimer;
	float centerTimer;
	float ringTimer;
	float smokeTimer;
	float startupTimer;
};