#include "asserv/effects.hpp"
#include <asserv/comm_asserv.hpp>
#include <asserv/pid.hpp>

#include <shared/utils.hpp>

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
		addTelem(&getPid(cl, i)->telem);
	addTelem(&this->cl->powerTelem);
}

CommAsserv::~CommAsserv() {
}

bool CommAsserv::handleCmd(uint8_t *data, size_t size) {
	if (Comm::handleCmd(data, size)) // Command handled by base class
		return true;

	uint8_t fbyte = data[0];

	uint8_t cmd = fbyte&0xF;
	uint8_t subcmd = (fbyte>>4)&0xF;

	//printf("handle size: %i\n", size);
	//printf("cmd: %i, subcmd:%i\n", cmd, subcmd);

	// Floats need to be aligned, can't just cast
	float f1, f2, f3, f4;
	int32_t is1, is2, is3, is4;
	uint32_t iu1, iu2;
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
				is3 = this->cl->encLeft->getSpeedCount();
				is4 = this->cl->encRight->getSpeedCount();
				this->sendDataSize = 4*sizeof(int32_t);
				memcpy(&this->sendData[4*0], &is1, sizeof(int32_t));
				memcpy(&this->sendData[4*1], &is2, sizeof(int32_t));
				memcpy(&this->sendData[4*2], &is3, sizeof(int32_t));
				memcpy(&this->sendData[4*3], &is4, sizeof(int32_t));
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
				memcpy(&f1, &data[6], sizeof(float));
				memcpy(&f2, &data[6+4], sizeof(float));
				this->effects->setControlState((ControlState)data[2]);
				this->effects->setBlinker((BlinkerState)data[3]);
				this->effects->setStop(data[1]&0x1);
				this->effects->setCenterStop((data[1]>>1)&0x1);
				this->effects->setHeadlights((HeadlightState)data[4]);
				this->effects->setRing((RingState)data[5]);
				this->effects->setDisco((data[1]>>2)&0x1);
				this->effects->setReversing((data[1]>>3)&0x1);
				this->effects->setSmoking((data[1]>>4)&0x1);
				this->effects->setPop(f1, f2);
			} else if (subcmd == 8) { // RGB debug
				memcpy(&iu1, &data[1], sizeof(uint32_t));
				memcpy(&iu2, &data[1+4], sizeof(uint32_t));
				this->effects->setControlState(ControlState::off);
				if (iu2 != 0xFFFFFFFF) {
					this->effects->leds->setColorRaw(iu2, iu1, data[9]);
				} else {
					this->effects->leds->setColor(iu1, data[9]);
				}
			} else if (subcmd == 9) { // PopUp servo debug
				memcpy(&f1, &data[1], sizeof(float));
				memcpy(&f2, &data[1+4], sizeof(float));
				if (f1 < -1 || f1 > 1) {
					this->effects->popup->left->disable();
				} else {
					this->effects->popup->left->setValue(f1);
				}
				if (f2 < -1 || f2 > 1) {
					this->effects->popup->right->disable();
				} else {
					this->effects->popup->right->setValue(f2);
				}
			} else if (subcmd == 10) { // LDR debug
				memset(&this->sendData[0], 0, sizeof(float)*2);
				if (this->effects->ldrExt) {
					f1 = this->effects->ldrExt->readLux();
					memcpy(&this->sendData[0], &f1, sizeof(float));
				}
				if (this->effects->ldrFront) {
					f2 = this->effects->ldrFront->readLux();
					memcpy(&this->sendData[4], &f2, sizeof(float));
				}
				this->sendDataSize = 2*sizeof(float);
			}
			break;
		case 14: // Additional HW CMD
			if (subcmd == 0) { // Get Battery stats
				f4 = calculateLipoPercentage(this->cl->lastPower.voltage);
				memcpy(&this->sendData[4*0], &this->cl->lastPower.voltage, sizeof(float));
				memcpy(&this->sendData[4*1], &this->cl->lastPower.current, sizeof(float));
				memcpy(&this->sendData[4*2], &this->cl->lastPower.power, sizeof(float));
				memcpy(&this->sendData[4*3], &f4, sizeof(float));
				this->sendDataSize = 4*sizeof(float);
			} else if (subcmd == 1) { // Get IMU data
				
			}
			break;
		default:
			return false;
	}
	return true;
}