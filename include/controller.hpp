#pragma once

#include <odometry.hpp>

class Controller
{
public:
	Controller(Odometry *odo);
	~Controller();

	float getDstTarget();
	float getAngleTarget();
	void gotoXY(float x, float y);
	void movePolar(float dst, float theta);
	void setTarget(float dst, float theta);
	void reset(float dst=0, float theta=0);

private:
	Odometry *odo;

	float targetDst;
	float targetTheta;
	float oldTargetTheta;
	float oldTargetDst;
};