#pragma  once

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

static inline float convertLight(char val) {
	return ((val - 'a') * 21.25f)/255.0f;
}

class Effects
{
public:
	Effects(ControlLoop *cl, LedProvider* prov, Piezo* piezo, Spoiler *spoiler, PopUp *popup);
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

	ControlState getControlState() {return this->controlState;}
	BlinkerState getBlinker() {return this->blinkers;}
	HeadlightState getHeadlights() {return this->headlights;}
	RingState getRingState() {return this->ringState;}
	bool getDisco() {return this->ringDisco;}
	bool getStop() {return this->stopping;}
	bool getCenterStop() {return this->stopCenter;}
	bool getReversing() {return this->reversing;}
	bool getSmoking() {return this->smoking;}

	// Handle all the stuff
	void work();

	LedProvider *leds;
	Piezo *piezo;
	Spoiler *spoiler;
	PopUp *popup;
private:
	ControlLoop *cl;
	absolute_time_t lastTime;

	long firstPixelHue;
	size_t chaseOffset;
	size_t wiperState;
	size_t smokeIdx;
	size_t smokeLen;

	ControlState controlState;
	BlinkerState blinkers;
	HeadlightState headlights;
	RingState ringState;
	bool stopping;
	bool stopCenter;
	bool reversing;
	bool ringDisco;
	bool smoking;

	float discoTimer;
	float blinkerTimer;
	float fancyBlinkerTimer;
	float centerTimer;
	float rainbowTimer;
	float smokeTimer;
};