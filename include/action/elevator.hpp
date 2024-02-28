#pragma once

#include <action/stepper_driver.hpp>
#include <action/endstop.hpp>

class Elevator
{
public:
	Elevator(StepperDriver *drv, Endstop *end, float mmPerTurn, float elevatorHeight, float elevatorMaxDst);
	~Elevator();

	void setEnable(bool en);
	bool isEnabled();

	void home();
	bool isHomed();

	float getPosition();

	// in mm, force to move when not homed
	void move(float dst, bool force=false);
	// in mm, absolute
	void moveTo(float pos);
	
private:

	void moveRaw(float dst);

	float toTurns(float dst);

	float pos; // in mm

	float mmPerTurn;
	float elevatorHeight;
	float elevatorMaxDst;

	bool state;
	bool homed;

	StepperDriver *drv;
	Endstop *end;
};