#pragma once

/*
	Small Heads-up:
	This is implemented using integration, which accumulates errors.
	Theses errors are as big as the timestep is small or too big, keep it reasonable.
	The ~0.2ms used in the asserv here seems to work pretty much perfectly.
	To improve it you could use doubles or use analytical expressions for the position.
	Keep in mind that there also is an error that accumulates in the time accumulator.
	For what we're doing, this is relatively fine.

	Note: It seems to struggle with very low values of float numbers not very well representable.
		For example 0.00001 will struggle a lot.

	Note 2: After some testing it looks like just changing time & pos to double fixes it mostly.
		We probably won't be doing it here though as it costs more ressources.
*/

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
	// Decelerates to a stop, if the acceleration is zero, the default maximum acceleration is used
	void stop(float acceleration = 0);
	// Applies the profile and returns the current position
	float process(float dt);
	// Returns the total time time taken to do the profile
	float getTotalTime();
	// Returns the current position in the profile
	float getPosition();
	// Returns the current velocity in the profile
	float getVelocity();
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
	// The deceleration value at which to stop, if non-zero then initiate deceleration
	float stopAmax;
	// The initial velocity at which the stop was initiated
	float stopVel;

	float target;

	float vmax, amax;
	float time, position, velocity;
	
};