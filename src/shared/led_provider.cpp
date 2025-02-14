#include <cstdint>
#include <cstdio>
#include <shared/led_provider.hpp>

// ====================
//  LedRange
// ====================


LedRange::Iterator LedRange::begin() {
	return this->itBeg.copy();
}
LedRange::Iterator LedRange::end() {
	return this->itEnd.copy();
}

LedRange::LedRange(LedProvider *prov, LedFunction fMask, LedPosition pMask, bool specific)
	: itBeg(prov, fMask, pMask, 0, specific), itEnd(prov, fMask, pMask, prov->getSize(), specific) {
}

LedRange::LedRange(std::vector<uint32_t> *leds)
 : itBeg(leds, 0), itEnd(leds, leds->size()) {

}

// ====================
//  LedRange Iterator
// ====================

LedRange::Iterator::Iterator(LedProvider* ptr, LedFunction fMask, LedPosition pMask, uint32_t pos, bool specific) 
: prov(ptr), fMask(fMask), pMask(pMask), pos(pos), specific(specific), leds(nullptr) {
	advanceToNext();
}

LedRange::Iterator::Iterator(std::vector<uint32_t> *leds, uint32_t pos)
: prov(nullptr), fMask(LedFunction::all), pMask(LedPosition::agnostic), pos(pos), specific(false), leds(leds) {
}

void LedRange::Iterator::advanceToNext() {
	uint32_t size = this->prov->getSize();
	if (this->specific) {
		while (this->pos < size
			&& ((this->prov->getLedFunction(this->pos) & this->fMask) != this->fMask || 
				(this->prov->getLedPosition(this->pos) & this->pMask) != this->pMask)) {
			this->pos++;
		}
	} else {
		while (this->pos < size
			&& ((this->prov->getLedFunction(this->pos) & this->fMask) == static_cast<LedFunction>(0) || 
				(this->prov->getLedPosition(this->pos) & this->pMask) == static_cast<LedPosition>(0))) {
			this->pos++;
		}
	}
}

LedRange::Iterator LedRange::Iterator::copy() {
	if (this->leds != nullptr)
		return LedRange::Iterator(this->leds, this->pos);
	return LedRange::Iterator(this->prov, this->fMask, this->pMask, this->pos, this->specific);
}

// ====================
//  Led Provider
// ====================

LedRange LedProvider::range(LedFunction fMask, LedPosition pMask, bool specific) {
	return LedRange(this, fMask, pMask, specific);
}

uint32_t LedProvider::getSizeParam(LedFunction fMask, LedPosition pMask, bool specific) {
	uint32_t count = 0;
	for (uint32_t __attribute__((unused)) idx : range(fMask, pMask, specific))
		count++;
	return count;
}

void LedProvider::setColor(uint32_t rgb, uint8_t brightness, LedFunction fMask, LedPosition pMask, bool specific) {
	for (uint32_t idx : range(fMask, pMask, specific))
		this->setColorRaw(idx, rgb, brightness);
}

void LedProvider::setLedParamsRange(uint32_t from, uint32_t to, LedFunction func, LedPosition pos) {
	for (uint32_t idx=from; idx<=to; idx++)
		this->setLedParams(idx, func, pos);
}


// ====================
//  Aggregate Provider
// ====================


void AggregateLedProvider::addProvider(LedProvider *prov) {
	uint32_t psize = prov->getSize();
	this->providers.push_back(LedProviderInfo(prov, this->size, psize));
	this->size += psize;
}

void AggregateLedProvider::removeProvider(LedProvider *prov) {
	//this->providers.erase(prov);
}

uint32_t AggregateLedProvider::getSize() {
	return this->size;
}

void AggregateLedProvider::setColorRaw(uint32_t idx, uint32_t rgb, uint8_t brightness) {
	LedProviderInfo pinf = getProvider(idx);
	if (!pinf.prov)
		return;
	pinf.prov->setColorRaw(idx-pinf.index, rgb, brightness);
}

void AggregateLedProvider::setLedParams(uint32_t idx, LedFunction func, LedPosition pos) {
	LedProviderInfo pinf = getProvider(idx);
	if (!pinf.prov)
		return;
	pinf.prov->setLedParams(idx-pinf.index, func, pos);
}

LedPosition AggregateLedProvider::getLedPosition(uint32_t idx) {
	LedProviderInfo pinf = getProvider(idx);
	if (!pinf.prov)
		return LedPosition::agnostic;
	return pinf.prov->getLedPosition(idx-pinf.index);

}
LedFunction AggregateLedProvider::getLedFunction(uint32_t idx) {
	LedProviderInfo pinf = getProvider(idx);
	if (!pinf.prov)
		return LedFunction::all;
	return pinf.prov->getLedFunction(idx-pinf.index);
}

void AggregateLedProvider::clear() {
	for (LedProviderInfo pinf : this->providers)
		pinf.prov->clear();
}

void AggregateLedProvider::display() {
	for (LedProviderInfo pinf : this->providers)
		pinf.prov->display();
}

LedProviderInfo AggregateLedProvider::getProvider(uint32_t idx) {
	if (idx >= this->size)
		return LedProviderInfo(nullptr, 0, 0);
	for (LedProviderInfo pinf : this->providers) {
		if (idx >= pinf.index && idx < pinf.index+pinf.size)
			return pinf;
	}
	return LedProviderInfo(nullptr, 0, 0);
}


// ====================
//  Cached Provider
// ====================


uint32_t CachedLedProvider::cacheId(LedFunction fMask, LedPosition pMask, bool specific) {
	return ((uint32_t)fMask) << 9 | ((uint32_t)pMask) << 1 | (specific & 1);
}

CachedLedProvider::CachedLedProvider(LedProvider &prov) : prov(prov) {
}

void CachedLedProvider::clearCache() {
	this->cache.clear();
}

// Led provider implementation

LedRange CachedLedProvider::range(LedFunction fMask, LedPosition pMask, bool specific) {
	uint32_t id = cacheId(fMask, pMask, specific);
	if (this->cache.contains(id))
		return LedRange(&this->cache.at(id).indices);

	LedRangeCache lrcache;
	lrcache.size = 0;

	for (uint32_t idx : LedRange((LedProvider*)this, fMask, pMask, specific)) {
		lrcache.indices.push_back(idx);
		lrcache.size++;
	}
	this->cache[id] = lrcache;
	return LedRange(&this->cache[id].indices);
}

uint32_t CachedLedProvider::getSize() {
	return this->prov.getSize();
}
uint32_t CachedLedProvider::getSizeParam(LedFunction fMask, LedPosition pMask, bool specific) {
	uint32_t id = cacheId(fMask, pMask, specific);
	if (this->cache.contains(id))
		return this->cache.at(id).size;
	return this->prov.getSizeParam(fMask, pMask, specific);

}
// Affects mask
void CachedLedProvider::setColorRaw(uint32_t idx, uint32_t rgb, uint8_t brightness) {
	this->prov.setColorRaw(idx, rgb, brightness);
}

void CachedLedProvider::setLedParams(uint32_t idx, LedFunction func, LedPosition pos) {
	this->prov.setLedParams(idx, func, pos);
}

LedPosition CachedLedProvider::getLedPosition(uint32_t idx) {
	return this->prov.getLedPosition(idx);
}
LedFunction CachedLedProvider::getLedFunction(uint32_t idx) {
	return this->prov.getLedFunction(idx);
}

void CachedLedProvider::clear() {
	this->prov.clear();
}

void CachedLedProvider::display() {
	this->prov.display();
}