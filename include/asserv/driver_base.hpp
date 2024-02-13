#pragma once

class DriverBase
{
public:
	// Duty cycle: + for forwards, - for backwards, 0 for idle (not braking)
	virtual void setPwm(float duty) = 0;
};