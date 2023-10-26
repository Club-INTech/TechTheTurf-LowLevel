#ifndef _DEFINE_SERVO_H
#define _DEFINE_SERVO_H

#include "types.h"
#include <stdint.h>

struct pid2
{
    decimal_t kp, ki, kd;
    decimal_t state_input[3];
    decimal_t state_output;
    uint64_t previous_time;
};

extern struct pid2 pid2_new(decimal_t kp, decimal_t ki, decimal_t kd);
extern decimal_t pid2_advance(struct pid2 *pid, decimal_t current_input);

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
