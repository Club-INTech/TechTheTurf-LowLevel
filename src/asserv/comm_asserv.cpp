#include "asserv/effects.hpp"
#include <asserv/comm_asserv.hpp>
#include <asserv/pid.hpp>

#if defined(ROBOT_MAIN) && !defined(ROBOT_MAIN_ODRIVE)
#include <asserv/driver_bg.hpp>
#endif

static inline PID *getPid(ControlLoop *cl, uint8_t idx) {
	switch (idx) {
		case 0:
			return cl->anglePid;
		case 1:
			return cl->dstPid;
		case 2:
			return cl->lSpeedPid;
		case 3:
			return cl->rSpeedPid;
		default:
			return nullptr;
	}
}

CommAsserv::CommAsserv(uint sdaPin, uint sclPin, uint addr, i2c_inst_t *i2c, ControlLoop *cl, Effects *eff) : Comm(sdaPin, sclPin, addr, i2c) {
	this->cl = cl;
	this->effects = eff;
	for (size_t i=0; i<4; i++)
		addTelem(i, &getPid(cl, i)->telem);
}

CommAsserv::~CommAsserv() {
}

void CommAsserv::handleCmd(uint8_t *data, size_t size) {
	uint8_t fbyte = data[0];

	uint8_t cmd = fbyte&0xF;
	uint8_t subcmd = (fbyte>>4)&0xF;

	//printf("handle size: %i\n", size);
	//printf("cmd: %i, subcmd:%i\n", cmd, subcmd);

	// Floats need to be aligned, can't just cast
	float f1, f2, f3, f4;
	int32_t is1, is2;
	TelemetryBase* telem;
	PID *pid;

	switch (cmd) {
		// Write operations, could be deferred from IRQ
		case 0:
			if (subcmd == 0) { // Turn ON/OFF
				if (data[1])
					this->cl->start();
				else
					this->cl->stop();
			} else if (subcmd == 1) { // Emergency stop
				this->cl->estop();
			}
			break;
		case 1: // Move
			memcpy(&f1, &data[1], sizeof(float));
			memcpy(&f2, &data[1+4], sizeof(float));
			//printf("dst %f theta %f\n", f1, f2);
			this->cl->ctrl->movePolar(f1, f2);
			break;
		case 5: // Change PID
			pid = getPid(this->cl, subcmd);
			memcpy(&f1, &data[1], sizeof(float));
			memcpy(&f2, &data[1+4], sizeof(float));
			memcpy(&f3, &data[1+4*2], sizeof(float));
			//printf("pid %i kp %f ki %f kd %f\n", subcmd, f1, f2, f3);
			pid->setPID(f1, f2, f3);
			break;
		case 6: // Telem on/off
			telem = getTelem(subcmd);
			if (!telem)
				break;

			if (data[1])
				telem->start();
			else
				telem->stop();
			break;
		case 9: // Set target
			memcpy(&f1, &data[1], sizeof(float));
			memcpy(&f2, &data[1+4], sizeof(float));
			//printf("dst %f theta %f\n", f1, f2);
			this->cl->ctrl->setTarget(f1, f2);
		case 13: // Write CL vars
			if (subcmd == 0) { // Dst SpeedProfile
				memcpy(&f1, &data[1], sizeof(float));
				memcpy(&f2, &data[1+4], sizeof(float));
				this->cl->ctrl->spDst->setVmax(f1);
				this->cl->ctrl->spDst->setAmax(f2);
			} else if (subcmd == 1) { // Angle SpeedProfile
				memcpy(&f1, &data[1], sizeof(float));
				memcpy(&f2, &data[1+4], sizeof(float));
				this->cl->ctrl->spAngle->setVmax(f1);
				this->cl->ctrl->spAngle->setAmax(f2);
			}
			break;
		// Read operations, can't be deferred
		case 2: // Get PID
			pid = getPid(this->cl, subcmd);
			this->sendDataSize = 3*sizeof(float);
			memcpy(&this->sendData[0], &pid->Kp, sizeof(float));
			memcpy(&this->sendData[4], &pid->Ki, sizeof(float));
			memcpy(&this->sendData[4*2], &pid->Kd, sizeof(float));
			break;
		case 3: // Get theta, rho
			if (subcmd == 0) {
				this->sendDataSize = 2*sizeof(float);
				memcpy(&this->sendData[0], &this->cl->odo->dst, sizeof(float));
				memcpy(&this->sendData[4], &this->cl->odo->theta, sizeof(float));
			} else if (subcmd == 1) {
				this->sendDataSize = 2*sizeof(float);
				memcpy(&this->sendData[0], &this->cl->odo->x, sizeof(float));
				memcpy(&this->sendData[4], &this->cl->odo->y, sizeof(float));
			}
			break;
		case 10: // Ready for next move
			this->sendDataSize = 1;
			this->sendData[0] = this->cl->ctrl->isReady();
			break;
		case 12: // Read CL vars
			if (subcmd == 0) { // Dst SpeedProfile
				this->sendDataSize = 2*sizeof(float);
				f1 = this->cl->ctrl->spDst->getVmax();
				f2 = this->cl->ctrl->spDst->getAmax();
				memcpy(&this->sendData[0], &f1, sizeof(float));
				memcpy(&this->sendData[4], &f2, sizeof(float));
			} else if (subcmd == 1) { // Angle SpeedProfile
				this->sendDataSize = 2*sizeof(float);
				f1 = this->cl->ctrl->spAngle->getVmax();
				f2 = this->cl->ctrl->spAngle->getAmax();
				memcpy(&this->sendData[0], &f1, sizeof(float));
				memcpy(&this->sendData[4], &f2, sizeof(float));
			}
			break;
		// Read & Write
		case 11: // Debug CMD
			if (subcmd == 0) { // Read encoders
				is1 = this->cl->encLeft->getCount();
				is2 = this->cl->encRight->getCount();
				this->sendDataSize = 2*sizeof(int32_t);
				memcpy(&this->sendData[0], &is1, sizeof(int32_t));
				memcpy(&this->sendData[4], &is2, sizeof(int32_t));
			} else if (subcmd == 1) { // Write raw motor values
				memcpy(&f1, &data[1], sizeof(float));
				memcpy(&f2, &data[1+4], sizeof(float));
				this->cl->drvLeft->setPwm(f1);
				this->cl->drvRight->setPwm(f2);
			} else if (subcmd == 2) { // Write raw asserv targets
				memcpy(&f1, &data[1], sizeof(float));
				memcpy(&f2, &data[1+4], sizeof(float));
				this->cl->ctrl->setRawTarget(f1, f2);
			} else if (subcmd == 3) { // Enable/Disable drivers
				this->cl->drvLeft->setEnable(data[1]);
				this->cl->drvRight->setEnable(data[1]);
			} else if (subcmd == 4) { // Get asserv state
				this->sendDataSize = 1;
				this->sendData[0] = this->cl->ctrl->getState();
			}
#if defined(ROBOT_MAIN) && !defined(ROBOT_MAIN_ODRIVE)
			else if (subcmd == 5) { // Get left BG stats
				((DriverBG*)this->cl->drvLeft)->bg->readStats(&f1, &f2, &f3, &f4);
				memcpy(&this->sendData[0], &f1, sizeof(float));
				memcpy(&this->sendData[4], &f2, sizeof(float));
				memcpy(&this->sendData[4*2], &f3, sizeof(float));
				memcpy(&this->sendData[4*3], &f4, sizeof(float));
				this->sendDataSize = 4*sizeof(float);
			} else if (subcmd == 6) { // Get right BG stats
				((DriverBG*)this->cl->drvRight)->bg->readStats(&f1, &f2, &f3, &f4);
				memcpy(&this->sendData[0], &f1, sizeof(float));
				memcpy(&this->sendData[4], &f2, sizeof(float));
				memcpy(&this->sendData[4*2], &f3, sizeof(float));
				memcpy(&this->sendData[4*3], &f4, sizeof(float));
				this->sendDataSize = 4*sizeof(float);
			}
#endif
			else if (subcmd == 7) { // Effects
				if (!this->effects)
					break;
				this->effects->setAuto(data[1]);
				this->effects->setBlinker((BlinkerState)data[2]);
				this->effects->setStop(data[3]);
				this->effects->setCenterStop(data[4]);
				this->effects->setHeadlights((HeadlightState)data[5]);
			}
			break;
		default:
			break;
	}
}