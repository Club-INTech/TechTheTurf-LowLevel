#ifndef _DEFINE_SERVO_H
#define _DEFINE_SERVO_H

#include "types.h"
#include <stdint.h>

struct pid
{
    decimal_t kp, ki, kd;
    uint64_t _last_time;
    decimal_t _last_error;
    decimal_t _error_integral;
};

extern struct pid pid_new(decimal_t kp, decimal_t ki, decimal_t kd);
extern decimal_t pid_advance(struct pid *pid, decimal_t error);

#endif // _DEFINE_SERVO_H
