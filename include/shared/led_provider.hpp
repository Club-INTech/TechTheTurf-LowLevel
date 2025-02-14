#pragma once

#include <shared/neopixel_connect.h>
#include <cstdint>
#include <vector>
#include <unordered_map>

enum class LedFunction : uint16_t {
	all = 0xFFFF,
	left = 1<<1,
	blinker = 1<<2,
	brakeLight = 1<<3,
	signalLight = 1<<4,
	headlight = 1<<5,
	ringLight = 1<<6,
	neonLight = 1<<7,
	fancyBlinker = 1<<8,
	reverseLight = 1<<9,
	smokeLight = 1<<10
};

enum class LedPosition : uint8_t {
	agnostic = 0xFF,
	left = 0x1,
	right = 0x2,
	center = 0x4,
	front = 0x8,
	rear = 0x10
};

inline constexpr LedPosition operator&(LedPosition x, LedPosition y) { return static_cast<LedPosition> (static_cast<uint32_t>(x) & static_cast<uint32_t>(y)); }
inline constexpr LedPosition operator|(LedPosition x, LedPosition y) { return static_cast<LedPosition> (static_cast<uint32_t>(x) | static_cast<uint32_t>(y)); }
inline constexpr LedPosition operator^(LedPosition x, LedPosition y) { return static_cast<LedPosition> (static_cast<uint32_t>(x) ^ static_cast<uint32_t>(y)); }
inline constexpr LedPosition operator~(LedPosition x) { return static_cast<LedPosition>(~static_cast<uint32_t>(x)); }
inline LedPosition &operator&=(LedPosition & x, LedPosition y) { x = x & y; return x; }
inline LedPosition &operator|=(LedPosition & x, LedPosition y) { x = x | y; return x; }
inline LedPosition &operator^=(LedPosition & x, LedPosition y) { x = x ^ y; return x; }
inline constexpr LedFunction operator&(LedFunction x, LedFunction y) { return static_cast<LedFunction> (static_cast<uint32_t>(x) & static_cast<uint32_t>(y)); }
inline constexpr LedFunction operator|(LedFunction x, LedFunction y) { return static_cast<LedFunction> (static_cast<uint32_t>(x) | static_cast<uint32_t>(y)); }
inline constexpr LedFunction operator^(LedFunction x, LedFunction y) { return static_cast<LedFunction> (static_cast<uint32_t>(x) ^ static_cast<uint32_t>(y)); }
inline constexpr LedFunction operator~(LedFunction x) { return static_cast<LedFunction>(~static_cast<uint32_t>(x)); }
inline LedFunction &operator&=(LedFunction & x, LedFunction y) { x = x & y; return x; }
inline LedFunction &operator|=(LedFunction & x, LedFunction y) { x = x | y; return x; }
inline LedFunction &operator^=(LedFunction & x, LedFunction y) { x = x ^ y; return x; }

class LedProvider;

class LedRange {
public:
	struct Iterator 
	{
		using iterator_category = std::forward_iterator_tag;
		using difference_type   = std::ptrdiff_t;
		using value_type        = uint32_t;
		using pointer           = uint32_t*;
		using reference         = uint32_t&;

		void advanceToNext();

		Iterator(LedProvider* ptr, LedFunction fMask=LedFunction::all, LedPosition pMask=LedPosition::agnostic, uint32_t pos=0, bool specific=false);
		Iterator(std::vector<uint32_t> *leds, uint32_t pos=0);

		Iterator copy();

		reference operator*() const { return (reference)(this->leds != nullptr ?  this->leds->at(this->pos) : this->pos); }
		pointer operator->() { return &(this->leds != nullptr ?  this->leds->at(this->pos) : this->pos); }
		Iterator& operator++() {
			this->pos++;
			if (this->leds == nullptr)
				advanceToNext();
			return *this;
		}
		friend bool operator==(const Iterator& a, const Iterator& b) { return a.leds == b.leds && a.pos == b.pos && a.prov == b.prov && a.fMask == b.fMask && a.pMask == b.pMask && a.specific == b.specific; };
		friend bool operator!=(const Iterator& a, const Iterator& b) { return a.leds != b.leds || a.pos != b.pos || a.prov != b.prov || a.fMask != b.fMask || a.pMask != b.pMask || a.specific != b.specific; };  

	private:
		LedProvider *prov;
		LedFunction fMask;
		LedPosition pMask;
		uint32_t pos;
		bool specific;
		std::vector<uint32_t> *leds;
	};

	LedRange(LedProvider *prov, LedFunction fMask, LedPosition pMask, bool specific);
	LedRange(std::vector<uint32_t> *leds);

	Iterator begin();
	Iterator end();
private:
	Iterator itBeg,itEnd;
};

class LedProvider {
public:
	virtual ~LedProvider() {};

	virtual LedRange range(LedFunction fMask=LedFunction::all, LedPosition pMask=LedPosition::agnostic, bool specific=false);

	virtual uint32_t getSize() = 0;
	virtual uint32_t getSizeParam(LedFunction fMask, LedPosition pMask=LedPosition::agnostic, bool specific=false);
	// Affects mask
	virtual void setColorRaw(uint32_t idx, uint32_t rgb, uint8_t brightness=255) = 0;
	virtual void setColor(uint32_t rgb, uint8_t brightness=255, LedFunction fMask=LedFunction::all, LedPosition pMask=LedPosition::agnostic, bool specific=false);

	virtual void setLedParams(uint32_t idx, LedFunction func, LedPosition pos) = 0;
	virtual void setLedParamsRange(uint32_t from, uint32_t to, LedFunction func, LedPosition pos);

	virtual LedPosition getLedPosition(uint32_t idx) = 0;
	virtual LedFunction getLedFunction(uint32_t idx) = 0;

	virtual void clear() = 0;

	virtual void display() = 0;
};

struct LedProviderInfo {
	LedProvider *prov;
	uint32_t index, size;

	LedProviderInfo(LedProvider *prov, uint32_t idx, uint32_t size) : prov(prov), index(idx), size(size) {}
};

class AggregateLedProvider : public LedProvider {
public:
	void addProvider(LedProvider *prov);
	void removeProvider(LedProvider *prov);	

	// Led provider implementation

	uint32_t getSize();
	// Affects mask
	void setColorRaw(uint32_t idx, uint32_t rgb, uint8_t brightness=255);

	void setLedParams(uint32_t idx, LedFunction func, LedPosition pos);

	LedPosition getLedPosition(uint32_t idx);
	LedFunction getLedFunction(uint32_t idx);

	void clear();

	void display();

private:
	LedProviderInfo getProvider(uint32_t idx);

	std::vector<LedProviderInfo> providers;
	uint32_t size;
};

struct LedRangeCache {
	std::vector<uint32_t> indices;
	uint32_t size;
};

class CachedLedProvider : public LedProvider {
public:
	CachedLedProvider(LedProvider &prov);

	void  __attribute__((optimize("O0"))) cacheRange(LedFunction fMask=LedFunction::all, LedPosition pMask=LedPosition::agnostic, bool specific=false)
			{LedRange __attribute__((unused)) rg = range(fMask, pMask, specific);}
	void clearCache();

	// Led provider implementation

	LedRange range(LedFunction fMask=LedFunction::all, LedPosition pMask=LedPosition::agnostic, bool specific=false);

	uint32_t getSize();
	uint32_t getSizeParam(LedFunction fMask, LedPosition pMask=LedPosition::agnostic, bool specific=false);
	// Affects mask
	void setColorRaw(uint32_t idx, uint32_t rgb, uint8_t brightness=255);

	void setLedParams(uint32_t idx, LedFunction func, LedPosition pos);

	LedPosition getLedPosition(uint32_t idx);
	LedFunction getLedFunction(uint32_t idx);

	void clear();

	void display();

private:
	static uint32_t cacheId(LedFunction fMask, LedPosition pMask=LedPosition::agnostic, bool specific=false);

	LedProvider &prov;

	std::unordered_map<uint32_t, LedRangeCache> cache;
};