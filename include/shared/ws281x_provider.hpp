#pragma once

#include <shared/led_provider.hpp>

struct LedParams {
	LedFunction func;
	LedPosition pos;
	bool invert_rg;
};

class WS281XProvider : public LedProvider {
public:
	WS281XProvider(uint8_t ws_pin, size_t count);
	size_t getSize();

	void setColorRaw(size_t idx, uint32_t rgb, uint8_t brightness);

	void setLedOrder(size_t idx, bool brg);
	void setLedOrderRange(size_t from, size_t to, bool brg);

	void setLedParams(size_t idx, LedFunction func, LedPosition pos);

	LedPosition getLedPosition(size_t idx);
	LedFunction getLedFunction(size_t idx);

	void clear();
	void display();

private:
	size_t size;
	NeoPixelConnect pixels;
	std::vector<LedParams> params;
};
