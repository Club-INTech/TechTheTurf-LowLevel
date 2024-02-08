#include <driver_bg.hpp>

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
	if (velocity == 0.0f) { 
		if (this->status) {
			this->bg->disable();
			this->status = false;
		}
		return;
	}

	if (!this->status) {
		this->bg->setMotionControl(MotionControlType::velocity);
		this->bg->enable();
		this->status = true;
	}

	if (this->reversed)
		velocity *= -1.0f;

	/*if (velocity > 0) {
		velocity += 8.0f;
	} else {
		velocity -= 8.0f;
	}*/

	this->bg->setTarget(velocity);
}