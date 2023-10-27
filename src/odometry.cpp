#include <odometry.hpp>
#include <math.h>

Odometry::Odometry(float encoderDistance) {
	this->encoderDistance = encoderDistance;
	reset();
}

Odometry::~Odometry() {

}

void Odometry::reset(float x, float y, float dst, float theta) {
	this->x = x;
	this->y = y;
	this->dst = dst;
	this->theta = theta;
}

// Cf. https://github.com/EsialRobotik/asserv_chibios/blob/master/src/Odometry.cpp
void Odometry::update(float deltaMovLeft, float deltaMovRight) {
	this->deltaDst = (deltaMovLeft + deltaMovRight) / 2.0f;
	float diffDst = deltaMovRight - deltaMovLeft;
	this->deltaTheta = diffDst / this->encoderDistance;

	if (diffDst == 0) { // Linear mouvement
		this->x += this->deltaDst * cos(theta);
		this->y += this->deltaDst * sin(theta);
	} else { // Arc mouvement
		// Radius of curvature
		float rCourb = this->deltaDst/this->deltaTheta;

		// Update distance
		this->dst += this->deltaDst;
		// Update angle
		this->theta += this->deltaTheta;

		// Update position
		this->x += rCourb * (-sin(theta) + sin(theta + this->deltaTheta));
		this->y += rCourb * ( cos(theta) - cos(theta + this->deltaTheta));
	}
}