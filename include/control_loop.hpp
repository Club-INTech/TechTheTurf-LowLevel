#pragma once

#include <pico/stdlib.h>
#include <pico/time.h>
#include <pico/sync.h>

#include <encoder.hpp>
#include <driver_base.hpp>
#include <pid.hpp>
#include <odometry.hpp>
#include <pll.hpp>
#include <controller.hpp>
#include <accel_limiter.hpp>

class ControlLoop
{
public:
	ControlLoop(Encoder *encLeft, Encoder *encRight, DriverBase *drvLeft, DriverBase *drvRight, Odometry *odo,
				PID *lSpeedPid, PID *rSpeedPid, PID *dstPid, PID *anglePid, PLL *lPll, PLL *rPll, 
				AccelLimiter *lSpeedTargetAlim, AccelLimiter *rSpeedTargetAlim, Controller *ctrl, float encoderWheelRadius, uint32_t positionLoopDownsample,
				float speedMultiplier);
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
	DriverBase *drvLeft;
	DriverBase *drvRight;
	Odometry *odo;

	PID *lSpeedPid;
	PID *rSpeedPid;
	PID *dstPid;
	PID *anglePid;
	PLL *lPll;
	PLL *rPll;
	AccelLimiter *lSpeedTargetAlim;
	AccelLimiter *rSpeedTargetAlim;

	Controller *ctrl;

	mutex_t mutex;

	float encoderWheelRadius;
	float speedMultiplier;
	uint32_t positionLoopDownsample;

	bool running;
};