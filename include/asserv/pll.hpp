#pragma once

#include <stdint.h>

class PLL
{

public:
    PLL(float bandwidth);

    void reset();
    void update(int32_t deltaPosition, float dt);
    void setBandwidth(float bandwidth);

    float position;
    float speed;

private:
    float kp;
    float ki;
    int64_t count;
};