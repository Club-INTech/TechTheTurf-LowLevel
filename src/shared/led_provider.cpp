#include <shared/led_provider.hpp>

LedRange::Iterator LedRange::begin() {
	return this->itBeg.copy();
}
LedRange::Iterator LedRange::end() {
	return this->itEnd.copy();
}

LedRange::LedRange(LedProvider *prov, LedFunction fMask, LedPosition pMask, bool specific)
	: itBeg(prov, fMask, pMask, 0, specific), itEnd(prov, fMask, pMask, prov->getSize(), specific) {
}

LedRange::Iterator::Iterator(LedProvider* ptr, LedFunction fMask, LedPosition pMask, size_t pos, bool specific) : prov(ptr), fMask(fMask), pMask(pMask), pos(pos), specific(specific) {
	advanceToNext();
}

void LedRange::Iterator::advanceToNext() {
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

LedRange::Iterator LedRange::Iterator::copy() {
	return LedRange::Iterator(this->prov, this->fMask, this->pMask, this->pos, this->specific);
}

LedRange LedProvider::range(LedFunction fMask, LedPosition pMask, bool specific) {
	return LedRange(this, fMask, pMask, specific);
}

size_t LedProvider::getSizeParam(LedFunction fMask, LedPosition pMask, bool specific) {
	size_t count = 0;
	for (size_t __attribute__((unused)) idx : range(fMask, pMask, specific)) {
		count++;
	}
	return count;
}

void LedProvider::setColor(uint32_t rgb, uint8_t brightness, LedFunction fMask, LedPosition pMask, bool specific) {
	for (size_t idx : range(fMask, pMask, specific))
		this->setColorRaw(idx, rgb, brightness);
}

void LedProvider::setLedParamsRange(size_t from, size_t to, LedFunction func, LedPosition pos) {
	for (size_t idx=from; idx<=to; idx++)
		this->setLedParams(idx, func, pos);
}