#include <shared/ws281x_provider.hpp>

WS281XProvider::WS281XProvider(uint8_t ws_pin, size_t count) : size(count), pixels(ws_pin, count, pio1, 0){
	this->params.resize(count);
	this->setLedParamsRange(0, count-1, LedFunction::all, LedPosition::agnostic);
}

size_t WS281XProvider::getSize() {
	return this->size;
}

void WS281XProvider::setColorRaw(size_t idx, uint32_t rgb, uint8_t brightness) {
	if (idx >= this->size)
		return;
	uint32_t r = (((rgb >> 16)&0xFF) * brightness) >> 8;
	uint32_t g = (((rgb >> 8)&0xFF) * brightness) >> 8;
	uint32_t b = (((rgb >> 0)&0xFF) * brightness) >> 8;
	bool invert_rg = this->params[idx].invert_rg;
	this->pixels.setPixel(idx, invert_rg ? g : r, invert_rg ? r : g, b, false);
}

void WS281XProvider::setLedOrder(size_t idx, bool brg) {
	if (idx >= this->size)
		return;
	this->params[idx].invert_rg = !brg;
}

void WS281XProvider::setLedOrderRange(size_t from, size_t to, bool brg) {
	for (size_t idx=from; idx <= to; idx++)
		this->setLedOrder(idx, brg);
}

void WS281XProvider::setLedParams(size_t idx, LedFunction func, LedPosition pos) {
	if (idx >= this->size)
		return;
	this->params[idx].func = func;
	this->params[idx].pos = pos;
}

LedPosition WS281XProvider::getLedPosition(size_t idx) {
	if (idx >= this->size)
		return LedPosition::agnostic;
	return this->params[idx].pos;
}

LedFunction WS281XProvider::getLedFunction(size_t idx) {
	if (idx >= this->size)
		return LedFunction::all;
	return this->params[idx].func;
}

void WS281XProvider::clear() {
	this->pixels.clear(false);
}

void WS281XProvider::display() {
	this->pixels.show();
}