#pragma once

#include <pico/stdlib.h>
#include <pico/time.h>
#include <encoder.hpp>
#include <driver.hpp>
#include <pid.hpp>
#include <odometry.hpp>
#include <pll.hpp>
#include <controller.hpp>
#include <accel_limiter.hpp>

class ControlLoop
{
public:
	ControlLoop(Encoder *encLeft, Encoder *encRight, Driver *drvLeft, Driver *drvRight, Odometry *odo,
				PID *lSpeedPid, PID *rSpeedPid, PID *dstPid, PID *anglePid, PLL *lPll, PLL *rPll, 
				AccelLimiter *dstAlim, AccelLimiter *angleAlim, Controller *ctrl, float encoderWheelRadius);
	~ControlLoop();

	void start();
	void stop();

	void work();

	int32_t lastCountLeft;
	int32_t lastCountRight;

	float lSpeedTarget;
	float rSpeedTarget;
	
	absolute_time_t lastTime;
	absolute_time_t lastTimePos;
	
	Encoder *encLeft;
	Encoder *encRight;
	Driver *drvLeft;
	Driver *drvRight;
	Odometry *odo;

	PID *lSpeedPid;
	PID *rSpeedPid;
	PID *dstPid;
	PID *anglePid;
	PLL *lPll;
	PLL *rPll;
	AccelLimiter *dstAlim;
	AccelLimiter *angleAlim;

	Controller *ctrl;

	float encoderWheelRadius;
	bool running;
};