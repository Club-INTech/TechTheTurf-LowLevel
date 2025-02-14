#include <shared/spoiler.hpp>

#include <stdio.h>

#include <math.h>
#include <cmath>

Spoiler::Spoiler(Servo *longServo, Servo *shortServo, SpeedProfile *spoilerHeightSp, SpeedProfile *spoilerAngleSp, float l1, float l2, float l3, float l4, float startH, float startAng)
	: longSrv(longServo), shortSrv(shortServo), heightSp(spoilerHeightSp), angleSp(spoilerAngleSp), l1(l1), l2(l2), l3(l3), l4(l4) {
	this->centered = false;
	this->hTarget = startH;
	this->angleTarget = startAng;
	this->newHTarget = 0;
	this->newAngleTarget = 0;
	this->heightSp->reset();
	this->angleSp->reset();
}

Spoiler::~Spoiler() {
	this->longSrv->disable();
	this->shortSrv->disable();
}

void Spoiler::moveTo(float h, float angle) {
	if (!this->finished()) {
		this->hTarget = this->heightSp->getPosition();
		this->angleTarget = this->angleSp->getPosition();
	}
	this->angleSp->initMove(angle - this->angleTarget);
	this->heightSp->initMove(h - this->hTarget);
	this->newHTarget = h;
	this->newAngleTarget = angle;
	this->centered = false;
}

bool Spoiler::finished() {
	return this->angleSp->isDone() && this->heightSp->isDone();
}

// Used to mount arms
void Spoiler::centerServos() {
	this->centered = true;
	this->hTarget = 0;
	this->angleTarget = 0;
	this->longSrv->setValue(0);
	this->shortSrv->setValue(0);
	this->heightSp->reset();
	this->angleSp->reset();
}

void Spoiler::work(float dt) {
	if (this->centered)
		return;

	if (this->finished())
		return;

	float ang = this->angleTarget + this->angleSp->process(dt);
	float height = this->hTarget + this->heightSp->process(dt);

	if (this->finished()) {
		this->hTarget = this->newHTarget;
		this->angleTarget = this->newAngleTarget;
	}

	// Shout out to Loic the GOAT for this
	//float longAng = std::fmod(0.0f,2.0f*M_PI);
	float longAng = std::asin(height / this->l1);

	float L5_sq = this->l2 * this->l2 + this->l1 * this->l1 - 2.0f * this->l2 * this->l1 * std::cos(longAng - ang);
	float alpha2 = std::acos(-0.5f * (this->l2 * this->l2 - this->l1 * this->l1 - L5_sq) / std::sqrt(L5_sq) / this->l1);
	float alpha3 = std::acos(-0.5f * (this->l3 * this->l3 - this->l4 * this->l4 - L5_sq) / std::sqrt(L5_sq) / this->l4);
	//float shortAng = std::fmod(2.0f*M_PI, 2.0f*M_PI);
	float shortAng = longAng + alpha2 + alpha3;

	float sval = (-shortAng) / M_PI;
	float lval = (longAng - M_PI) / M_PI;
	this->shortSrv->setValue(sval);
	this->longSrv->setValue(lval);

	printf("s=%f l=%f, h=%f ang=%f\n", sval, lval, height, ang);
}