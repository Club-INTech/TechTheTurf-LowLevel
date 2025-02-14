#pragma once

#include <shared/servo.hpp>

class PopUp
{
public:
	PopUp(Servo *left, Servo *right, float closeValue, float openValue, float rightOffset);
	~PopUp();

	void setPop(float left, float right);

	void setOpen(bool open);

	Servo *left, *right;
private:
	float mapValue(float val);

	float clVal, opVal;
	float rightOffset;
};