#pragma once

class AccelLimiter
{
public:
	AccelLimiter(float max);
	~AccelLimiter();

	float limit(float val, float dt);
	void reset();

private:
	float limitValue(float val, float dt);

	float maxAccel;
	float lastValue;
};