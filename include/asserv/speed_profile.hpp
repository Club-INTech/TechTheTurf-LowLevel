#pragma once

class SpeedProfile
{
public:
	SpeedProfile(float vmax, float amax);
	~SpeedProfile();

	void setVmax(float vmax);
	void setAmax(float amax);
	float getVmax();
	float getAmax();

	// Starts a move 
	void initMove(float distance);
	// Applies the profile and returns the current position
	float process(float dt);
	// Returns the total time time taken to do the profile
	float getTotalTime();
	// Returns the current position in the profile
	float getPosition();
	bool isDone();

	void reset();

private:

	void updateLimits();

	bool done;
	bool trap;

	// The minimum distance required to be able to do a trap.
	// else the profile is a triangle 
	float trapDist;

	float direction;
	// The maximum velocity that the triangle hit, if on a triangle profile
	float vmaxTrig;
	// ta is the accel and decel time
	// tc is the time spent at constant velocity (if on a trap. profile)
	float ta, tc;
	float target;

	float vmax, amax;
	float time, pos;
	
};