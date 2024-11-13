#pragma once

#include <shared/neopixel_connect.h>
#include <cstdint>
#include <vector>

enum class LedFunction : uint8_t {
	all = 0xFF,
	left = 0x1,
	blinker = 0x2,
	brakeLight = 0x4,
	signalLight = 0x8,
	headlight = 0x10,
	ringLight = 0x20,
	neonLight = 0x40,
	fancyBlinker = 0x80
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

class LedProvider;

class LedRange {
public:
	struct Iterator 
	{
		using iterator_category = std::forward_iterator_tag;
		using difference_type   = std::ptrdiff_t;
		using value_type        = size_t;
		using pointer           = size_t*;
		using reference         = size_t&;

		void advanceToNext();

		Iterator(LedProvider* ptr, LedFunction fMask=LedFunction::all, LedPosition pMask=LedPosition::agnostic, size_t pos=0, bool specific=false);

		Iterator copy();

		reference operator*() const { return (reference)this->pos; }
		pointer operator->() { return &this->pos; }
		Iterator& operator++() {
			this->pos++;
			advanceToNext();
			return *this;
		}
		friend bool operator==(const Iterator& a, const Iterator& b) { return a.prov == b.prov && a.fMask == b.fMask && a.pMask == b.pMask && a.pos == b.pos && a.specific == b.specific; };
		friend bool operator!=(const Iterator& a, const Iterator& b) { return a.prov != b.prov || a.fMask != b.fMask || a.pMask != b.pMask || a.pos != b.pos || a.specific != b.specific; };  

	private:
		LedProvider *prov;
		LedFunction fMask;
		LedPosition pMask;
		size_t pos;
		bool specific;
	};

	LedRange(LedProvider *prov, LedFunction fMask, LedPosition pMask, bool specific);

	Iterator begin();
	Iterator end();
private:
	Iterator itBeg,itEnd;
};

class LedProvider {
public:
	virtual ~LedProvider() {};

	LedRange range(LedFunction fMask=LedFunction::all, LedPosition pMask=LedPosition::agnostic, bool specific=false);

	virtual size_t getSize() = 0;
	virtual size_t getSizeParam(LedFunction fMask, LedPosition pMask=LedPosition::agnostic, bool specific=false);
	// Affects mask
	virtual void setColorRaw(size_t idx, uint32_t rgb, uint8_t brightness=255) = 0;
	virtual void setColor(uint32_t rgb, uint8_t brightness=255, LedFunction fMask=LedFunction::all, LedPosition pMask=LedPosition::agnostic, bool specific=false);

	virtual void setLedParams(size_t idx, LedFunction func, LedPosition pos) = 0;
	virtual void setLedParamsRange(size_t from, size_t to, LedFunction func, LedPosition pos);

	virtual LedPosition getLedPosition(size_t idx) = 0;
	virtual LedFunction getLedFunction(size_t idx) = 0;

	virtual void clear() = 0;

	virtual void display() = 0;
};