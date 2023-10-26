#ifndef _DEFINE_MOTOR_H
#define _DEFINE_MOTOR_H

#include "types.h"

#include <stdint.h>

#define LEFT    0
#define RIGHT   1

struct motor
{
    uint8_t pins[2];
    uint8_t slice;
    uint8_t direction;
    decimal_t cyclical_report;
};

extern struct motor motor_new(uint8_t pin_left, uint8_t pin_right);
extern void motor_set_rotation(struct motor *motor, decimal_t rotation);
extern void motor_dispatch(struct motor *motor);

#endif // _DEFINE_MOTOR_H
