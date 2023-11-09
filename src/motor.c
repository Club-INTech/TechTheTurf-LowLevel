#include "motor.h"
#include "types.h"

#include <math.h>
#include "stdio.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"

#define WRAP_VAL 256

struct motor motor_new(uint8_t pin_left, uint8_t pin_right)
{
    uint8_t slice = pwm_gpio_to_slice_num(pin_left);
    if (slice != pwm_gpio_to_slice_num(pin_right))
    {
        printf("FAIS CHIER MEC METS DEUX PINS DE LA MEME SLICE PUTAIN DE MERDE\n");
        while (1);
    }

    struct motor motor;
    motor.slice = slice;
    motor.direction = LEFT;
    motor.cyclical_report = 0;

    // gpio_init(pin_left);
    // gpio_init(pin_right);
    // gpio_set_dir(pin_left, GPIO_OUT);
    // gpio_set_dir(pin_right, GPIO_OUT);
    // gpio_init(slice * 2);
    // gpio_init(slice * 2 + 1);

    gpio_set_function(pin_left, GPIO_FUNC_PWM);
    gpio_set_function(pin_right, GPIO_FUNC_PWM);

    pwm_set_wrap(slice, WRAP_VAL);

    return motor;
}


static inline decimal_t clampf(decimal_t a, decimal_t min, decimal_t max)
{ 
    return (a < min) ? min : (a > max ? max : a);
}

void motor_set_rotation(struct motor *motor, decimal_t rotation)
{
    motor->direction = rotation > 0.0;
    rotation = rotation > 0.0 ? rotation : -rotation;
    motor->cyclical_report = clampf(rotation, 0.0, 1.0);
}

void motor_dispatch(struct motor *motor)
{
    uint8_t cyclical_report = (uint8_t)(motor->cyclical_report * (WRAP_VAL - 1));
    pwm_set_chan_level(motor->slice, PWM_CHAN_A, motor->direction ? 0 : cyclical_report);
    pwm_set_chan_level(motor->slice, PWM_CHAN_B, motor->direction ? cyclical_report : 0);
    pwm_set_enabled(motor->slice, true);
}


