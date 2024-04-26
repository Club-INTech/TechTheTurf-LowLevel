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
	Controller(Odometry *odo, SpeedProfile *spDst, SpeedProfile *spAngle, float dstTolerance, float angleTolerance, float amaxDstEstop, float amaxAngleEstop);
	~Controller();

	float getDstTarget();
	float getAngleTarget();

	void work(float dt);

	void estop();

	bool isReady();
	ControllerState getState();
	void movePolar(float dst, float theta);
	void setTarget(float dst, float theta);
	void setRawTarget(float dst, float theta);
	void reset(float dst=0, float theta=0);

	Odometry *odo;
	SpeedProfile *spDst;
	SpeedProfile *spAngle;

private:
	float dstTolerance;
	float angleTolerance;
	float amaxDstEstop;
	float amaxAngleEstop;

	Target oldTarget;
	Target target;
	ControllerState state;
};