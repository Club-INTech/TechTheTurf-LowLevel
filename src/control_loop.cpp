#include <control_loop.hpp>
#include <stdio.h>
#include <math.h>

ControlLoop::ControlLoop(Encoder *encLeft, Encoder *encRight, Driver *drvLeft, Driver *drvRight, Odometry *odo,
				PID *lSpeedPid, PID *rSpeedPid, PID *dstPid, PID *anglePid, PLL *lPll, PLL *rPll, 
				AccelLimiter *dstAlim, AccelLimiter *angleAlim, Controller *ctrl, float encoderWheelRadius) {
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
	this->dstAlim = dstAlim;
	this->angleAlim = angleAlim;
	this->ctrl = ctrl;

	this->encoderWheelRadius = encoderWheelRadius;

	this->lSpeedPid->setClamp(-1.0f, 1.0f);
	this->rSpeedPid->setClamp(-1.0f, 1.0f);

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
	this->dstAlim->reset();
	this->angleAlim->reset();
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
	static uint counter = 0;

	if (!this->running)
		return;

	mutex_try_enter(&this->mutex, nullptr);

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
	float lCurrentSpeed = this->encLeft->convertRevolutions(this->lPll->speed) * 2.0f * M_PI * this->encoderWheelRadius;
	float rCurrentSpeed = this->encRight->convertRevolutions(this->rPll->speed) * 2.0f * M_PI * this->encoderWheelRadius;

	// Update last counts
	this->lastCountLeft = lCnt;
	this->lastCountRight = rCnt;

	// Update odometry
	this->odo->update(lDetaDst, rDetaDst);

	// Position asserv
	if (counter == 4) {
		counter = 0;

		// Delta Time of pos asserv
		float dtPos = ((float)absolute_time_diff_us(this->lastTimePos, time))/1e6;
		this->lastTimePos = time;
		
		// Calculate virtual polar motor targets
		float dstSpeedTarget = this->dstPid->calculate(this->ctrl->getDstTarget(), this->odo->dst, dtPos);
		float angleSpeedTarget = this->anglePid->calculate(this->ctrl->getAngleTarget(), this->odo->theta, dtPos);
		//float dstSpeedTarget = dstAlim->limit(this->dstPid->calculate(this->ctrl->getDstTarget(), this->odo->dst, dtPos), dtPos);
		//float angleSpeedTarget = angleAlim->limit(this->anglePid->calculate(this->ctrl->getAngleTarget(), this->odo->theta, dtPos), dtPos);
		
		// Real motor speed targets
		this->lSpeedTarget = dstSpeedTarget - angleSpeedTarget;
		this->rSpeedTarget = dstSpeedTarget + angleSpeedTarget;
	}

	//this->rSpeedTarget = 60.0f * (((float)std::min(counter,50u))/50.0f) + -120.0f * (((float)std::min(std::max(((int)counter)-200,0),50))/50.0f);
	//this->lSpeedTarget = 60.0f * (((float)std::min(counter,50u))/50.0f) + -120.0f * (((float)std::min(std::max(((int)counter)-200,0),50))/50.0f);

	// Final control values for motors
	float lSpeed = this->lSpeedPid->calculate(this->lSpeedTarget, lCurrentSpeed, dt);
	float rSpeed = this->rSpeedPid->calculate(this->rSpeedTarget, rCurrentSpeed, dt);

	//printf("cl%f cr%f tl%f tr%f sl%f sr%f x%f y%f d%f t%f dt%f\n", lSpeed, rSpeed, this->lSpeedTarget, this->rSpeedTarget, lCurrentSpeed, rCurrentSpeed, this->odo->x, this->odo->y, this->odo->dst, this->odo->theta, dt*1000.0f);

	// Write speed to motors
	this->drvLeft->setPwm(lSpeed);
	this->drvRight->setPwm(rSpeed);

	counter++;
	mutex_exit(&this->mutex);
}