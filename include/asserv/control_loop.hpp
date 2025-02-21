#pragma once

#include <shared/ina236.hpp>
#include <shared/telemetry.hpp>
#include <pico/stdlib.h>
#include <pico/time.h>
#include <pico/sync.h>

#include <asserv/encoder.hpp>
#include <asserv/driver_base.hpp>
#include <asserv/pid.hpp>
#include <asserv/odometry.hpp>
#include <asserv/pll.hpp>
#include <asserv/controller.hpp>
#include <asserv/accel_limiter.hpp>

struct PowerTelemData
{
	float voltage;
	float current;
	float power;

	PowerTelemData(float volts=0, float curr=0, float pow=0) : voltage(volts), current(curr), power(pow) {
	}

	static TelemetryPacketType type() {return TelemetryPacketType::Power;}
};

class ControlLoop
{
public:
	ControlLoop(Encoder *encLeft, Encoder *encRight, DriverBase *drvLeft, DriverBase *drvRight, Odometry *odo,
				PID *lSpeedPid, PID *rSpeedPid, PID *dstPid, PID *anglePid, PLL *lPll, PLL *rPll, 
				AccelLimiter *lSpeedTargetAlim, AccelLimiter *rSpeedTargetAlim, Controller *ctrl, float encoderWheelRadius, uint32_t positionLoopDownsample,
				INA236 *ina=nullptr);
	~ControlLoop();

	void start();
	void stop();

	// Emergency Stop
	void estop();

	void work();

	int32_t lastCountLeft;
	int32_t lastCountRight;

	float lSpeedTarget;
	float rSpeedTarget;

	float lCurrentSpeed;
	float rCurrentSpeed;
	
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

	INA236 *ina236;
	PowerTelemData lastPower;
	Telemetry<PowerTelemData> powerTelem;

	Controller *ctrl;

	mutex_t mutex;

	float encoderWheelRadius;
	uint32_t positionLoopDownsample;

	float lastDt;

	bool running;
};