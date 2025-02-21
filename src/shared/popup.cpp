#include <algorithm>
#include <shared/popup.hpp>

PopUp::PopUp(Servo *left, Servo *right, float closeValue, float openValue, float rightOffset)
: left(left), right(right), clVal(closeValue), opVal(openValue), rightOffset(rightOffset) {
	setOpen(false);
}

void PopUp::setPop(float left, float right) {
	this->left->setValue(mapValue(left));
	this->right->setValue(-mapValue(right) + this->rightOffset);
}

void PopUp::setOpen(bool open) {
	float val = open ? this->opVal : this->clVal;
	this->left->setValue(val);
	this->right->setValue(-val + this->rightOffset);
}

float PopUp::mapValue(float val) {
	val = std::clamp(val, 0.0f, 1.0f);
	return this->clVal + val*(this->opVal-this->clVal);
}