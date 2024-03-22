#pragma once

class DriverBase
{
public:
	// Duty cycle: + for forwards, - for backwards, 0 for braking
	virtual void setPwm(float duty) = 0;
	// Turn on / off the drivers
	virtual void setEnable(bool enabled);
};