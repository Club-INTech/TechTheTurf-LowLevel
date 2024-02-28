#include <action/elevator.hpp>

Elevator::Elevator(StepperDriver *drv, Endstop *end, float mmPerTurn, float elevatorHeight, float elevatorMaxDst) {
	this->drv = drv;
	this->end = end;
	this->mmPerTurn = mmPerTurn;
	this->elevatorHeight = elevatorHeight;
	this->elevatorMaxDst = elevatorMaxDst;
	this->pos = 0;
	this->state = false;
	this->homed = false;
}

Elevator::~Elevator() {

}

bool Elevator::isHomed() {
	return this->homed;
}

bool Elevator::isEnabled() {
	return this->state;
}

float Elevator::getPosition() {
	return this->pos;
}

void Elevator::setEnable(bool en) {
	this->state = en;
	this->drv->setEnable(en);
	// We disabled the drivers, we're not homed anymore
	if (!en) {
		this->homed = false;
		this->pos = 0;
	}
}

float Elevator::toTurns(float dst) {
	return dst / this->mmPerTurn;
}

void Elevator::moveRaw(float dst) {
	this->drv->rotateTurns(this->toTurns(dst));
}

void Elevator::home() {
	if (!this->state)
		return;

	uint32_t maxSteps = this->drv->toSteps(this->toTurns(this->elevatorMaxDst));

	for (uint32_t step=0;step<maxSteps;step++) {
		if (this->end->poll())
			break;
		this->drv->stepCount(-1);
	}

	this->moveRaw(5.0f);

	for (uint32_t step=0;step<maxSteps;step++) {
		if (this->end->poll())
			break;
		this->drv->stepCount(-1);
	}

	this->homed = true;
	this->pos = 0;
}

void Elevator::move(float dst, bool force) {
	if (!this->state)
		return;

	if (!force && !this->homed)
		return;

	if (this->homed) {
		float target = this->pos + dst;
	
		if (target < 0)
			dst = 0-this->pos;
	
		if (target > this->elevatorMaxDst)
			dst = this->elevatorMaxDst-this->pos;
	
		this->pos += dst;
	}

	this->moveRaw(dst);
}

void Elevator::moveTo(float pos) {
	this->move(pos - this->pos);
}