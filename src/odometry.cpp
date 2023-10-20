#include <odometry.hpp>
#include <math.h>

Odometry::Odometry(float encoderDistance) {
	this->encoderDistance = encoderDistance;
	reset();
}

Odometry::~Odometry() {

}

void Odometry::reset(float x, float y, float theta) {
	this->x = x;
	this->y = y;
	this->theta = theta;
}

// Cf. https://github.com/EsialRobotik/asserv_chibios/blob/master/src/Odometry.cpp
void Odometry::update(float deltaMovLeft, float deltaMovRight) {
	this->deltaDst = (deltaMovLeft + deltaMovRight) / 2.0f;
	float diffDst = deltaMovRight - deltaMovLeft;
	this->deltaTheta = diffDst / this->encoderDistance;

	if (diffDst == 0) { // Mvmt linéaire
		this->x += this->deltaDst * cos(theta);
		this->y += this->deltaDst * sin(theta);
	} else { // Arc de cercle
		// Rayon de courbure
		float rCourb = this->deltaDst/this->deltaTheta;

		// Mise à jour position
		this->x += rCourb * (-sin(theta) + sin(theta + this->deltaTheta));
		this->y += rCourb * ( cos(theta) - cos(theta + this->deltaTheta));
		// Mise à jour angle
		this->theta += this->deltaTheta;

		// Clamp PI
		if (this->theta > M_PI)
			this->theta -= 2 * M_PI;
		else if (this->theta <= M_PI)
			this->theta += 2 * M_PI;
	}
}