#pragma once

#include <asserv/odometry.hpp>
#include <asserv/speed_profile.hpp>

struct Target
{
	float dst;
	float theta;

	void set(float dst, float theta) {
		this->dst = dst;
		this->theta = theta;
	}

	void reset() {
		set(0, 0);
	}
};

enum ControllerState
{
	reachingTheta,
	reachingDst,
	reachedTarget
};

class Controller
{
public:
	Controller(Odometry *odo, SpeedProfile *sp);
	~Controller();

	float getDstTarget();
	float getAngleTarget();

	void work(float dt);

	bool isReady();
	void movePolar(float dst, float theta);
	void setTarget(float dst, float theta);
	void reset(float dst=0, float theta=0);

	Odometry *odo;
	SpeedProfile *sp;

private:
	Target oldTarget;
	Target target;
	ControllerState state;
};