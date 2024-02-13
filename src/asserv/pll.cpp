#include <math.h>

#include <asserv/pll.hpp>

PLL::PLL(float bandwidth) {
	setBandwidth(bandwidth);
	reset();
}

void PLL::reset() {
	this->position = 0.0f;
	this->speed = 0.0f;
	this->count = 0;
}

// Look at this thread for more informations
// https://discourse.odriverobotics.com/t/rotor-encoder-pll-and-velocity/224

void PLL::update(int32_t deltaTicks, float dt) {
	float cval = ((float)deltaTicks)/dt;
	this->speed += (cval - this->speed)/3.0f;

	/*
	// Prediction
	this->position += dt * this->speed;

	// Compute error between prediction and encoder information
	this->count += deltaTicks;
	float deltaPos = (float) (this->count - (int64_t) floor(this->position));

	// PLL correction
	this->position += dt * this->kp * deltaPos;
	this->speed += dt * this->ki * deltaPos;

	if (abs(this->speed) < 0.5f * dt * this->ki)
		this->speed = 0.0f;*/
}

void PLL::setBandwidth(float bandwidth) {
	this->kp = 2.0f * bandwidth;
	this->ki = 0.25f * this->kp * this->kp;
}