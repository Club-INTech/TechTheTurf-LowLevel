#pragma once

#include <asserv/driver_base.hpp>
#include <asserv/comm_odrive.hpp>

class DriverODrive : public DriverBase
{
public:
	DriverODrive(CommODrive *odrive, uint8_t axis, bool reversed);
	~DriverODrive();

	void setPwm(float velocity);

private:
	uint8_t axis;
	bool reversed;

	CommODrive *odrive;
	bool status;
};