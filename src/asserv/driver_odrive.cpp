#include <asserv/driver_odrive.hpp>
#include <cmath>

DriverODrive::DriverODrive(CommODrive *odrive, uint8_t axis, bool reversed) {
	this->odrive = odrive;
	this->reversed = reversed;
	this->axis = axis;
	this->status = false;
	this->odrive->setAxisState(this->axis, ODrive::AxisState::AXIS_STATE_IDLE);
}


DriverODrive::~DriverODrive() {

}

void DriverODrive::setPwm(float velocity) {
	if (velocity == 0.0f) { 
		if (this->status) {
			this->odrive->setAxisState(this->axis, ODrive::AxisState::AXIS_STATE_IDLE);
			this->status = false;
		}
		return;
	}

	if (!this->status) {
		this->odrive->setAxisState(this->axis, ODrive::AxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
		this->odrive->setAxisControlMode(this->axis, ODrive::ControlMode::CONTROL_MODE_VELOCITY_CONTROL);
		this->status = true;
	}

	if (this->reversed)
		velocity *= -1.0f;

	this->odrive->setTargetVelocity(this->axis, velocity / (2.0f * M_PI));
}
