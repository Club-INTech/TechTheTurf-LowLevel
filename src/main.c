#include "motor.h"
#include "servo.h"
#include "incremental_encoder.h"
#include "types.h"

#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include <time.h>
#include <math.h>
#include <stdio.h>
#include "pico/time.h"

struct motor motors[2];
struct pid pid_theta;
struct pid pid_rho;
struct incremental_encoder encoders[2];


#define MOTOR_PIN_LEFT_FORWARD      2
#define MOTOR_PIN_LEFT_BACKWARD     3
#define MOTOR_PIN_RIGHT_FORWARD     4
#define MOTOR_PIN_RIGHT_BACKWARD    5

#define INITIAL_PID_THETA_KP 1.0
#define INITIAL_PID_THETA_KI 0.0
#define INTIIAL_PID_THETA_KD 0.0

#define INITIAL_PID_RHO_KP 1.0
#define INITIAL_PID_RHO_KI 0.0
#define INTIIAL_PID_RHO_KD 0.0

void setup()
{
    stdio_init_all();
    motors[LEFT] = motor_new(MOTOR_PIN_LEFT_FORWARD, MOTOR_PIN_LEFT_BACKWARD);
    motors[RIGHT] = motor_new(MOTOR_PIN_RIGHT_FORWARD, MOTOR_PIN_RIGHT_BACKWARD);
    attach_encoder();
//    pid[0] = pid _new(PID_KP, PID_KI, PID_KD);
//    pid[1] = pid_new(PID_KP, PID_KI, PID_KD);

    pid_theta = pid_new(INITIAL_PID_THETA_KP, INITIAL_PID_THETA_KI, INITIAL_PID_THETA_KD);
    pid_rho = pid_new(INITIAL_PID_RHO_KP, INITIAL_PID_RHO_KI, INITIAL_PID_RHO_KD);

//    gpio_init(0);
//    attach_encoder(NULL, 0);
}

volatile decimal_t position_ticks = 0.0;
volatile decimal_t angular_ticks = 0.0;
volatile uint64_t prev_time = 0;

void loop()
{

    int32_t left_ticks = get_coder_left();
    int32_t right_ticks = get_coder_right();

    decimal_t speed_ticks = (decimal_t)(left_ticks + right_ticks) / 2.0;
    decimal_t angular_speed_ticks = (decimal_t)(right_ticks - left_ticks);

    uint64_t cur_time = time_us_64();
    decimal_t dt = (cur_time - prev_time) / 1000000.0;

    position_ticks += dt * speed_ticks;
    angular_ticks += dt * angular_speed_ticks;

    

    motor_set_rotation(&motors[LEFT], (float)get_coder_left() / 100);
    motor_set_rotation(&motors[RIGHT], (float)get_coder_left() / 100);
    motor_dispatch(&motors[LEFT]);
    motor_dispatch(&motors[RIGHT]);
}


int main()
{
    setup();
    while (1)
        loop();

    return 0;
}
