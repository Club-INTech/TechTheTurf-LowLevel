#pragma once

#include <shared/servo.hpp>
#include <asserv/speed_profile.hpp>

class Spoiler
{
public:
	Spoiler(Servo *longServo, Servo *shortServo, SpeedProfile *spoilerHeightSp, SpeedProfile *spoilerAngleSp, float l1, float l2, float l3, float l4, float startH=0, float startAng=0);
	~Spoiler();

	void moveTo(float h, float angle);

	// Used to mount arms
	void centerServos();
	bool finished();

	void work(float dt);

private:
	Servo *longSrv,*shortSrv;

	bool centered;
	float hTarget, angleTarget;
	float newHTarget, newAngleTarget;

	SpeedProfile *heightSp, *angleSp;
	float l1,l2,l3,l4;
};