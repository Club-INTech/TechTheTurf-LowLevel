#include <asserv/driver_bg.hpp>

DriverBG::DriverBG(CommBG *bg, bool reversed) {
	this->bg = bg;
	this->reversed = reversed;
	this->bg->disable();
	this->status = false;
}

DriverBG::~DriverBG() {
	this->bg->disable();
}

// The PID is clamped to max velocity rather than max duty 
void DriverBG::setPwm(float velocity) {
	if (!this->status)
		return;

	if (this->reversed)
		velocity *= -1.0f;

	/*if (velocity > 0) {
		velocity += 8.0f;
	} else {
		velocity -= 8.0f;
	}*/

	this->bg->setTarget(velocity);
}

void DriverBG::setEnable(bool enabled) {
	if (!this->status && enabled) {
		this->bg->setMotionControl(MotionControlType::velocity);
		this->bg->enable();
		this->status = true;
	} else if (this->status && !enabled) {
		this->bg->disable();
		this->status = false;
	}
}