#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <asserv/control_loop.hpp>

ControlLoop::ControlLoop(Encoder *encLeft, Encoder *encRight, DriverBase *drvLeft, DriverBase *drvRight, Odometry *odo,
				PID *lSpeedPid, PID *rSpeedPid, PID *dstPid, PID *anglePid, PLL *lPll, PLL *rPll, 
				AccelLimiter *lSpeedTargetAlim, AccelLimiter *rSpeedTargetAlim, Controller *ctrl, float encoderWheelRadius, uint32_t positionLoopDownsample,
				float speedMultiplier) {
	this->encLeft = encLeft;
	this->encRight = encRight;
	this->drvLeft = drvLeft;
	this->drvRight = drvRight;
	this->odo = odo;

	this->lSpeedPid = lSpeedPid;
	this->rSpeedPid = rSpeedPid;
	this->dstPid = dstPid;
	this->anglePid = anglePid;
	this->lPll = lPll;
	this->rPll = rPll;
	this->lSpeedTargetAlim = lSpeedTargetAlim;
	this->rSpeedTargetAlim = rSpeedTargetAlim;
	this->ctrl = ctrl;

	this->encoderWheelRadius = encoderWheelRadius;
	this->positionLoopDownsample = positionLoopDownsample;
	this->speedMultiplier = speedMultiplier;

	this->lastTime = get_absolute_time();
	this->lastTimePos = this->lastTime;
	
	this->lastCountLeft = 0;
	this->lastCountRight = 0;
	this->lSpeedTarget = 0;
	this->rSpeedTarget = 0;
	this->running = false;

	mutex_init(&this->mutex);
}

ControlLoop::~ControlLoop() {
	drvLeft->setPwm(0.0);
	drvRight->setPwm(0.0);
}

void ControlLoop::start() {
	if (this->running)
		return;
	mutex_try_enter(&this->mutex, nullptr);
	absolute_time_t time = get_absolute_time();
	this->lastTime = time;
	this->lastTimePos = time;
	this->encLeft->reset();
	this->encRight->reset();
	this->odo->reset();
	this->ctrl->reset();
	this->dstPid->reset();
	this->anglePid->reset();
	this->lSpeedPid->reset();
	this->rSpeedPid->reset();
	this->lSpeedTargetAlim->reset();
	this->rSpeedTargetAlim->reset();
	this->lPll->reset();
	this->rPll->reset();
	this->lastCountLeft = 0;
	this->lastCountRight = 0;
	this->running = true;
	mutex_exit(&this->mutex);
}

void ControlLoop::stop() {
	if (!this->running)
		return;
	mutex_try_enter(&this->mutex, nullptr);
	this->running = false;
	this->drvLeft->setPwm(0.0f);
	this->drvRight->setPwm(0.0f);
	mutex_exit(&this->mutex);
}

void ControlLoop::work() {
	static uint32_t counter = 0;

	if (!this->running)
		return;

	mutex_try_enter(&this->mutex, nullptr);

	ctrl->work();

	// Calculate Delta time & update last time
	absolute_time_t time = get_absolute_time();
	float dt = ((float)absolute_time_diff_us(this->lastTime, time))/((float)1e6);
	this->lastTime = time;

	// Get encoders counts
	int32_t lCnt = this->encLeft->getCount();
	int32_t rCnt = this->encRight->getCount();

	// Calc. delta dist from counts
	float lDetaDst = this->encLeft->convertRevolutions(lCnt - this->lastCountLeft) * 2.0f * M_PI * this->encoderWheelRadius;
	float rDetaDst = this->encRight->convertRevolutions(rCnt - this->lastCountRight) * 2.0f * M_PI * this->encoderWheelRadius;

	this->lPll->update(lCnt - this->lastCountLeft, dt);
	this->rPll->update(rCnt - this->lastCountRight, dt);

	// Estimate current speed
	float lCurrentSpeed = this->encLeft->convertRevolutions(this->lPll->speed) * 2.0f * M_PI * this->encoderWheelRadius * this->speedMultiplier;
	float rCurrentSpeed = this->encRight->convertRevolutions(this->rPll->speed) * 2.0f * M_PI * this->encoderWheelRadius * this->speedMultiplier;

	// Update last counts
	this->lastCountLeft = lCnt;
	this->lastCountRight = rCnt;

	// Update odometry
	this->odo->update(lDetaDst, rDetaDst);

	// Position asserv
	if (counter >= this->positionLoopDownsample) {
		counter = 0;

		// Delta Time of pos asserv
		float dtPos = ((float)absolute_time_diff_us(this->lastTimePos, time))/1e6;
		this->lastTimePos = time;
		
		// Calculate virtual polar motor targets
		//float dstSpeedTarget = this->dstPid->calculate(this->ctrl->getDstTarget(), this->odo->dst, dtPos);
		//float angleSpeedTarget = this->anglePid->calculate(this->ctrl->getAngleTarget(), this->odo->theta, dtPos);

		float dstSpeedTarget = std::clamp(this->dstPid->calculate(this->ctrl->getDstTarget(), this->odo->dst, dtPos), -3000.0f, 3000.0f);
		float angleSpeedTarget = std::clamp(this->anglePid->calculate(this->ctrl->getAngleTarget(), this->odo->theta, dtPos), -100000.0f, 100000.0f);
		
		// Real motor speed targets
		this->lSpeedTarget = this->lSpeedTargetAlim->limit(dstSpeedTarget - angleSpeedTarget, dtPos);
		this->rSpeedTarget = this->rSpeedTargetAlim->limit(dstSpeedTarget + angleSpeedTarget, dtPos);
	}

	// Final control values for motors
	float lSpeed = this->lSpeedPid->calculate(this->lSpeedTarget, lCurrentSpeed, dt);
	float rSpeed = this->rSpeedPid->calculate(this->rSpeedTarget, rCurrentSpeed, dt);

	// Write speed to motors
	this->drvLeft->setPwm(lSpeed);
	this->drvRight->setPwm(rSpeed);

	//printf("cl%f cr%f tl%f tr%f sl%f sr%f x%f y%f d%f t%f dt%f\n", lSpeed, rSpeed, this->lSpeedTarget, this->rSpeedTarget, lCurrentSpeed, rCurrentSpeed, this->odo->x, this->odo->y, this->odo->dst, this->odo->theta, dt*1000.0f);

	counter++;
	mutex_exit(&this->mutex);
}