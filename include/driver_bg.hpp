#pragma once

#include <driver_base.hpp>
#include <comm_bg.hpp>

class DriverBG : public DriverBase
{
public:
	DriverBG(CommBG *bg, bool reversed);
	~DriverBG();

	void setPwm(float duty);

private:
	bool reversed;

	CommBG *bg;
	bool status;
};