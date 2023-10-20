#include <control_loop.hpp>
#include <stdio.h>
#include <math.h>

#define WHEEL_RADIUS (68.0f/2.0f)


ControlLoop::ControlLoop(Encoder *encLeft, Encoder *encRight, Driver *drvLeft, Driver *drvRight, Odometry *odo,
				PID *lSpeedPid, PID *rSpeedPid, PID *dstPid, PID *anglePid, PLL *lPll, PLL *rPll, 
				AccelLimiter *lAlim, AccelLimiter *rAlim, Controller *ctrl) {
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
	this->lAlim = lAlim;
	this->rAlim = rAlim;
	this->ctrl = ctrl;

	this->lSpeedPid->setClamp(-1.0f, 1.0f);
	this->rSpeedPid->setClamp(-1.0f, 1.0f);

	this->lastTime = get_absolute_time();
	this->lastTimePos = this->lastTime;
	
	this->lastCountLeft = 0;
	this->lastCountRight = 0;
	this->lSpeedTarget = 0;
	this->rSpeedTarget = 0;
	//setTarget(0);
}

ControlLoop::~ControlLoop() {
	drvLeft->setPwm(0.0);
	drvRight->setPwm(0.0);
}

void ControlLoop::work() {
	static uint counter = 0;

	// Calculate Delta time & update last time
	absolute_time_t time = get_absolute_time();
	float dt = ((float)absolute_time_diff_us(this->lastTime, time))/((float)1e6);
	this->lastTime = time;

	// Get encoders counts
	int32_t lCnt = this->encLeft->getCount();
	int32_t rCnt = this->encRight->getCount();

	// Calc. delta dist from counts
	float lDetaDst = this->encLeft->convertRevolutions(lCnt - this->lastCountLeft) * 2.0f * M_PI * WHEEL_RADIUS;
	float rDetaDst = this->encRight->convertRevolutions(rCnt - this->lastCountRight) * 2.0f * M_PI * WHEEL_RADIUS;

	this->lPll->update(lCnt - this->lastCountLeft, dt);
	this->rPll->update(rCnt - this->lastCountRight, dt);

	// Estimate current speed
	float lCurrentSpeed = this->encLeft->convertRevolutions(this->lPll->speed) * 2.0f * M_PI * WHEEL_RADIUS;
	float rCurrentSpeed = this->encRight->convertRevolutions(this->rPll->speed) * 2.0f * M_PI * WHEEL_RADIUS;

	// Update last counts
	this->lastCountLeft = lCnt;
	this->lastCountRight = rCnt;

	// Update odometry
	this->odo->update(lDetaDst, rDetaDst);

	// Update Polar PIDs
	this->dstPid->accumulate(this->odo->deltaDst);
	this->anglePid->accumulate(this->odo->deltaTheta);

	// Position asserv
	if (counter == 4) {
		counter = 0;

		// Delta Time of pos asserv
		float dtPos = ((float)absolute_time_diff_us(this->lastTimePos, time))/1e6;
		this->lastTimePos = time;
		
		// Calculate virtual polar motor targets
		float dstSpeedTarget = this->dstPid->calculateAcc(this->ctrl->getDstTarget(), dtPos);
		float angleSpeedTarget = this->anglePid->calculateAcc(this->ctrl->getAngleTarget(), dtPos);
		
		// Real motor speed targets
		this->lSpeedTarget = lAlim->limit(dstSpeedTarget - angleSpeedTarget, dtPos);
		this->rSpeedTarget = rAlim->limit(dstSpeedTarget + angleSpeedTarget, dtPos);
	}

	//this->rSpeedTarget = 60.0f * (((float)std::min(counter,50u))/50.0f) + -120.0f * (((float)std::min(std::max(((int)counter)-200,0),50))/50.0f);
	//this->lSpeedTarget = 60.0f * (((float)std::min(counter,50u))/50.0f) + -120.0f * (((float)std::min(std::max(((int)counter)-200,0),50))/50.0f);

	// Final control values for motors
	float lSpeed = this->lSpeedPid->calculate(this->lSpeedTarget, lCurrentSpeed, dt);
	float rSpeed = this->rSpeedPid->calculate(this->rSpeedTarget, rCurrentSpeed, dt);

	printf("cl%f cr%f tl%f tr%f sl%f sr%f x%f y%f t%f dt%f\n", lSpeed, rSpeed, this->lSpeedTarget, this->rSpeedTarget, lCurrentSpeed, rCurrentSpeed, this->odo->x, this->odo->y, this->odo->theta, dt*1000.0f);

	// Write speed to motors
	this->drvLeft->setPwm(lSpeed);
	this->drvRight->setPwm(rSpeed);

	counter++;
}