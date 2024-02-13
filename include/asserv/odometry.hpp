#pragma once

#include <stdint.h>

class Odometry
{
public:
	Odometry(float encoderDistance);
	~Odometry();

	void reset(float x = 0.0f, float y = 0.0f, float dst = 0.0f, float theta = 0.0f);
	void update(float deltaMovLeft, float deltaMovRight);

	// Current accumulated estimated position
	// In cartesian
	float x;
	float y;
	// In polar
	float dst;
	float theta;

	// Last update's delta values
	float deltaDst;
	float deltaTheta;
private:
	float encoderDistance;
};