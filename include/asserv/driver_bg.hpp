#pragma once

#include <asserv/driver_base.hpp>
#include <asserv/comm_bg.hpp>

class DriverBG : public DriverBase
{
public:
	DriverBG(CommBG *bg, bool reversed);
	~DriverBG();

	void setPwm(float duty);
	void setEnable(bool enabled);

private:
	bool reversed;

	CommBG *bg;
	bool status;
};