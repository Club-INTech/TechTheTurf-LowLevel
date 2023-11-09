#include "motor.h"
#include "servo.h"
#include "incremental_encoder.h"
#include "types.h"

#include <pico/stdlib.h>
#include <hardware/gpio.h>

#include <time.h>
#include <math.h>
#include <stdio.h>
#include "pico/time.h"

#define LEFT            0
#define RIGHT           1
#define DISTANCE        0
#define ORIENTATION     1

#define PI 3.14159265358979323846264338327

#define MOTOR_PIN_LEFT_FORWARD      20
#define MOTOR_PIN_LEFT_BACKWARD     21
#define MOTOR_PIN_RIGHT_FORWARD     27
#define MOTOR_PIN_RIGHT_BACKWARD    26

#define INITIAL_PID_DISTANCE_KP 1.0
#define INITIAL_PID_DISTANCE_KI 0.0
#define INITIAL_PID_DISTANCE_KD 0.0

#define INITIAL_PID_ORIENTATION_KP 1.0
#define INITIAL_PID_ORIENTATION_KI 0.0
#define INITIAL_PID_ORIENTATION_KD 0.0

#define TICKS_PER_TERN 1024
#define ENCODER_DIAMETER 34e-3
#define DISTANCE_BETWEEN_ENCODERS = 86.8e-3

struct motor motors[2];
struct pid pid[2];

void setup()
{
    stdio_init_all();
    motors[LEFT] = motor_new(MOTOR_PIN_LEFT_FORWARD, MOTOR_PIN_LEFT_BACKWARD);
    motors[RIGHT] = motor_new(MOTOR_PIN_RIGHT_FORWARD, MOTOR_PIN_RIGHT_BACKWARD);

    pid[DISTANCE] = pid_new(INITIAL_PID_ORIENTATION_KP, INITIAL_PID_ORIENTATION_KI, INITIAL_PID_ORIENTATION_KD);
    pid[ORIENTATION] = pid_new(INITIAL_PID_DISTANCE_KP, INITIAL_PID_DISTANCE_KI, INITIAL_PID_DISTANCE_KD);

    attach_encoder();

//    gpio_init(0);
//    attach_encoder(NULL, 0);
}

void loop()
{

    int32_t left_ticks = get_coder_left();
    int32_t right_ticks = get_coder_right();

    decimal_t distance = (decimal_t)(left_ticks + right_ticks) * PI * ENCODER_DIAMETER / DISTANCE_BETWEEN_ENCODERS / 2.0;
    decimal_t orientation = (decimal_t)(right_ticks - left_ticks) * PI ( ENCODER_DIAMETER / DISTANCE_BETWEEN_ENCODERS;

    decimal_t target_distance = 10.f;
    decimal_t target_orientation = 10.f;

    decimal_t pid_distance = pid_advance(&pid[DISTANCE], target_distance - distance);
    decmial_t orientation_delta = target_orientation - orientation;
    orientation_delta += PI;
    orientation_delta %= 2.0 * PI;
    orientation_delta -= PI;
    decimal_t pid_orientation = pid_advance(&pid[ORIENTATION], orientation_delta);

    motor_set_rotation(&motors[LEFT], pid_distance - pid_orientation);
    motor_set_rotation(&motors[RIGHT], pid_distance + pid_orientation);
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
