#pragma  once

#include "action/dynamixel_xl430.hpp"
#include <asserv/control_loop.hpp>
#include <shared/neopixel_connect.h>
#include <cstdint>
#include <unordered_map>
#include <vector>

#define BLINKER_PERIOD 0.25f
#define CENTER_PERIOD 0.15f

#define CENTER_DIM 20

#define BRAKE_DIM 25
#define BRAKE_RGB 0xFF0000

#define HEADLIGHTS_DIM 30
#define HEADLIGHTS_RGB 0xA6D3F5

#define RING_BRIGHTNESS 150

#define BLINKER_RGB 0xFF4000

enum class ControlState {
	automatic = 0,
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
	speed
};

enum class LedFunction : uint8_t {
	all = 0xFF,
	left = 0x1,
	blinker = 0x2,
	brakeLight = 0x4,
	signalLight = 0x8,
	headlight = 0x10,
	ringLight = 0x20,
	neonLight = 0x30,
};

enum class LedPosition : uint8_t {
	agnostic = 0xFF,
	left = 0x1,
	right = 0x2,
	center = 0x4,
	front = 0x8,
	rear = 0x10
};

inline constexpr LedPosition operator&(LedPosition x, LedPosition y) { return static_cast<LedPosition> (static_cast<size_t>(x) & static_cast<size_t>(y)); }
inline constexpr LedPosition operator|(LedPosition x, LedPosition y) { return static_cast<LedPosition> (static_cast<size_t>(x) | static_cast<size_t>(y)); }
inline constexpr LedPosition operator^(LedPosition x, LedPosition y) { return static_cast<LedPosition> (static_cast<size_t>(x) ^ static_cast<size_t>(y)); }
inline constexpr LedPosition operator~(LedPosition x) { return static_cast<LedPosition>(~static_cast<size_t>(x)); }
inline LedPosition &operator&=(LedPosition & x, LedPosition y) { x = x & y; return x; }
inline LedPosition &operator|=(LedPosition & x, LedPosition y) { x = x | y; return x; }
inline LedPosition &operator^=(LedPosition & x, LedPosition y) { x = x ^ y; return x; }
inline constexpr LedFunction operator&(LedFunction x, LedFunction y) { return static_cast<LedFunction> (static_cast<size_t>(x) & static_cast<size_t>(y)); }
inline constexpr LedFunction operator|(LedFunction x, LedFunction y) { return static_cast<LedFunction> (static_cast<size_t>(x) | static_cast<size_t>(y)); }
inline constexpr LedFunction operator^(LedFunction x, LedFunction y) { return static_cast<LedFunction> (static_cast<size_t>(x) ^ static_cast<size_t>(y)); }
inline constexpr LedFunction operator~(LedFunction x) { return static_cast<LedFunction>(~static_cast<size_t>(x)); }
inline LedFunction &operator&=(LedFunction & x, LedFunction y) { x = x & y; return x; }
inline LedFunction &operator|=(LedFunction & x, LedFunction y) { x = x | y; return x; }
inline LedFunction &operator^=(LedFunction & x, LedFunction y) { x = x ^ y; return x; }

class LedProvider {
public:
	struct Iterator 
	{
		using iterator_category = std::forward_iterator_tag;
		using difference_type   = std::ptrdiff_t;
		using value_type        = size_t;
		using pointer           = size_t*;
		using reference         = size_t&;

		void advanceToNext() {
			if (this->specific) {
				while (this->pos < this->prov->getSize()
					&& ((this->prov->getLedFunction(this->pos) & this->fMask) != this->fMask || 
						(this->prov->getLedPosition(this->pos) & this->pMask) != this->pMask)) {
					this->pos++;
				}
			} else {
				while (this->pos < this->prov->getSize()
					&& ((this->prov->getLedFunction(this->pos) & this->fMask) == static_cast<LedFunction>(0) || 
						(this->prov->getLedPosition(this->pos) & this->pMask) == static_cast<LedPosition>(0))) {
					this->pos++;
				}
			}
		}

		Iterator(LedProvider* ptr, LedFunction fMask=LedFunction::all, LedPosition pMask=LedPosition::agnostic, size_t pos=0, bool specific=false) : prov(ptr), fMask(fMask), pMask(pMask), pos(pos), specific(specific) {
			advanceToNext();
		}

		reference operator*() const { return (reference)this->pos; }
		pointer operator->() { return &this->pos; }
		Iterator& operator++() {
			this->pos++;
			advanceToNext();
			return *this;
		}
		friend bool operator== (const Iterator& a, const Iterator& b) { return a.prov == b.prov && a.fMask == b.fMask && a.pMask == b.pMask && a.pos == b.pos; };
		friend bool operator!= (const Iterator& a, const Iterator& b) { return a.prov != b.prov || a.fMask != b.fMask || a.pMask != b.pMask || a.pos != b.pos; };  

	private:
		LedProvider *prov;
		LedFunction fMask;
		LedPosition pMask;
		size_t pos;
		bool specific;
	};

	Iterator begin(LedFunction fMask=LedFunction::all, LedPosition pMask=LedPosition::agnostic, bool specific=false) { return Iterator(this, fMask, pMask, 0, specific); }
	Iterator end(LedFunction fMask=LedFunction::all, LedPosition pMask=LedPosition::agnostic, bool specific=false)   { return Iterator(this, fMask, pMask, getSize()-1, specific); }

	virtual ~LedProvider() {};

	virtual size_t getSize() = 0;
	virtual size_t getSizeParam(LedFunction fMask, LedPosition pMask=LedPosition::agnostic, bool specific=false) {
		size_t count = 0;
		for (LedProvider::Iterator it = this->begin(fMask, pMask, specific); it != this->end(fMask, pMask, specific); ++it)
			count++;
		return count;
	}

	// Affects mask
	virtual void setColorRaw(size_t idx, uint32_t rgb, uint8_t brightness=255) = 0;
	virtual void setColor(uint32_t rgb, uint8_t brightness=255, LedFunction fMask=LedFunction::all, LedPosition pMask=LedPosition::agnostic, bool specific=false) {
		for (LedProvider::Iterator it = this->begin(fMask, pMask, specific); it != this->end(fMask, pMask, specific); ++it)
			this->setColorRaw(*it, rgb, brightness);
	}

	virtual void setLedParams(size_t idx, LedFunction func, LedPosition pos) = 0;
	virtual void setLedParamsRange(size_t from, size_t to, LedFunction func, LedPosition pos) {
		for (size_t idx=from; idx<=to; idx++)
			this->setLedParams(idx, func, pos);
	}

	virtual LedPosition getLedPosition(size_t idx) = 0;
	virtual LedFunction getLedFunction(size_t idx) = 0;

	virtual void clear() = 0;

	virtual void display() = 0;
};

struct LedParams {
	LedFunction func;
	LedPosition pos;
	bool invert_rg;
};

class WS281XProvider : public LedProvider {
public:
	WS281XProvider(uint8_t ws_pin, size_t count) : size(count), pixels(ws_pin, count, pio1, 0){
		this->params.resize(count);
		this->setLedParamsRange(0, count-1, LedFunction::all, LedPosition::agnostic);
	}

	size_t getSize() {
		return this->size;
	}

	void setColorRaw(size_t idx, uint32_t rgb, uint8_t brightness) {
		if (idx >= this->size)
			return;
		uint32_t r = (((rgb >> 16)&0xFF) * brightness) >> 8;
		uint32_t g = (((rgb >> 8)&0xFF) * brightness) >> 8;
		uint32_t b = (((rgb >> 0)&0xFF) * brightness) >> 8;
		bool invert_rg = this->params[idx].invert_rg;
		this->pixels.setPixel(idx, invert_rg ? g : r, invert_rg ? r : g, b, false);
	}

	void setLedOrder(size_t idx, bool brg) {
		if (idx >= this->size)
			return;
		this->params[idx].invert_rg = !brg;
	}

	void setLedOrderRange(size_t from, size_t to, bool brg) {
		for (size_t idx=from; idx <= to; idx++)
			this->setLedOrder(idx, brg);
	}

	void setLedParams(size_t idx, LedFunction func, LedPosition pos) {
		if (idx >= this->size)
			return;
		LedParams params = {func, pos};
		this->params[idx] = params;
	}

	LedPosition getLedPosition(size_t idx) {
		if (idx >= this->size)
			return LedPosition::agnostic;
		return this->params[idx].pos;
	}

	LedFunction getLedFunction(size_t idx) {
		if (idx >= this->size)
			return LedFunction::all;
		return this->params[idx].func;
	}

	void clear() {
		this->pixels.clear(false);
	}

	void display() {
		this->pixels.show();
	}
private:
	size_t size;
	NeoPixelConnect pixels;
	std::vector<LedParams> params;
};

class Effects
{
public:
	Effects(ControlLoop *cl, LedProvider* prov, uint8_t center_brake_pin);
	~Effects();

	void setControlState(ControlState state) {this->controlState = state;}
	void setBlinker(BlinkerState state) {this->blinkers = state;}
	void setRing(RingState state) {this->ringState = state;}
	void setStop(bool en) {this->stopping = en;}
	void setCenterStop(bool en) {this->stopCenter = en;}
	void setHeadlights(HeadlightState state) {this->headlights = state;}

	ControlState getControlState() {return this->controlState;}
	BlinkerState getBlinker() {return this->blinkers;}
	HeadlightState getHeadlights() {return this->headlights;}
	RingState getRingState() {return this->ringState;}
	bool getStop() {return this->stopping;}
	bool getCenterStop() {return this->stopCenter;}

	// Handle all the stuff
	void work();

private:
	LedProvider *leds;
	ControlLoop *cl;
	uint8_t center_brake_pin;
	absolute_time_t lastTime;

	long firstPixelHue;

	ControlState controlState;
	BlinkerState blinkers;
	HeadlightState headlights;
	RingState ringState;
	bool stopping;
	bool stopCenter;

	float blinkerTimer;
	float centerTimer;
	float rainbowTimer;
};