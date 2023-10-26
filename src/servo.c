#include "servo.h"

#include "pico/time.h"

struct pid2 pid2_new(decimal_t kp, decimal_t ki, decimal_t kd)
{
    struct pid2 pid2;
    pid2.kp = kp;
    pid2.ki = ki;
    pid2.kd = kd;
    pid2.state_input[0] = 0.0;
    pid2.state_input[1] = 0.0;
    pid2.state_input[2] = 0.0;
    pid2.state_output = 0.0;
    pid2.previous_time = time_us_64();

    return pid2;
}

decimal_t pid2_advance(struct pid2 *pid2, decimal_t current_input)
{
    pid2->state_input[2] = pid2->state_input[1];
    pid2->state_input[1] = pid2->state_input[0];
    pid2->state_input[0] = current_input;

    uint64_t new_time = time_us_64();
    decimal_t dt = (decimal_t)(new_time - pid2->previous_time) / 1000000.0;
    pid2->previous_time = new_time;

    decimal_t P = pid2->kd + pid2->kp * dt + pid2->ki * dt * dt;
    decimal_t Q = 2.0 * pid2->kd + pid2->kp * dt;

    return pid2->state_output += 1.0 / dt * (P * pid2->state_input[0]
            + Q * pid2->state_input[1]
            + pid2->kd * pid2->state_input[2]);

}

struct pid pid_new(decimal_t kp, decimal_t ki, decimal_t kd)
{
    return (struct pid)
    {
        .kp = kp,
        .ki = ki,
        .kd = kd,
        ._last_time = time_us_64(),
        ._last_error = (decimal_t)0.0,
        ._error_integral = (decimal_t)0.0
    };
}

decimal_t pid_advance(struct pid *pid, decimal_t error)
{
    uint64_t new_time = time_us_64();
    decimal_t dt = (decimal_t)(new_time - pid->_last_time) / 1000000.0;
    pid->_last_time = new_time;

    pid->_error_integral += error * dt;
    decimal_t error_derivitive = (error - pid->_last_error) / dt;
    pid->_last_error = error;

    decimal_t kp = pid->kp * error;
    decimal_t ki = pid->ki * pid->_error_integral;
    decimal_t kd = pid->kd * error_derivitive;
}
