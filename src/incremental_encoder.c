#include "incremental_encoder.h"

#include "hardware/gpio.h"
#include "hardware/irq.h"
#include <stdio.h>

#define ENCODER_PIN_LEFT_A      6
#define ENCODER_PIN_LEFT_B      7
#define ENCODER_PIN_RIGHT_A     8
#define ENCODER_PIN_RIGHT_B     9


#define ENCODER_PIN_LEFT_A_MASK         (1 << ENCODER_PIN_LEFT_A)
#define ENCODER_PIN_LEFT_B_MASK         (1 << ENCODER_PIN_LEFT_B)
#define ENCODER_PIN_RIGHT_A_MASK        (1 << ENCODER_PIN_RIGHT_A)
#define ENCODER_PIN_RIGHT_B_MASK        (1 << ENCODER_PIN_RIGHT_B)
#define ENCODER_PIN_MASK                (ENCODER_PIN_LEFT_A_MASK | ENCODER_PIN_LEFT_B_MASK | ENCODER_PIN_RIGHT_A_MASK | ENCODER_PIN_RIGHT_B_MASK)

volatile int32_t coder_left = 0;
volatile int32_t coder_right = 0;

int32_t get_coder_left(void)
{
    return coder_left;
}

int32_t get_coder_right(void)
{
    return coder_right;
}


void interrupt_callback(uint gpio, uint32_t callback)
{

    /*
    
    If the two pins are different or equal, the direction should change.
    The same can be said with whether it is the first or the second pin that
    initialized the callback

    #########################
    | 0 0 | 0 0 | 1 1 | 1 1 |
    |-l---|---l-|-h---|---h-| 
    | <-- | --> | <-- | --> |
    -------------------------
    | 0 1 | 0 1 | 1 0 | 1 0 |
    |-l---|--h--|-h---|---l-|
    | --> | <-- | --> | <-- |
    #########################
    */

    uint32_t gpio_state = gpio_get_all() & ENCODER_PIN_MASK;

    if (gpio == ENCODER_PIN_LEFT_A || ENCODER_PIN_LEFT_B)
    {
        int8_t is_pin_a = (gpio == ENCODER_PIN_LEFT_A) ? 1 : -1;
        int8_t are_pins_different = (!!(gpio_state & ENCODER_PIN_LEFT_A_MASK) ^ !!(gpio_state & ENCODER_PIN_LEFT_B_MASK)) ? 1 : -1;
        coder_left += is_pin_a * are_pins_different;
    }
    else if (gpio == ENCODER_PIN_RIGHT_A || ENCODER_PIN_RIGHT_B)
    {
        int8_t is_pin_a = (gpio == ENCODER_PIN_RIGHT_A) ? 1 : -1;
        int8_t are_pins_different = (!!(gpio_state & ENCODER_PIN_RIGHT_A_MASK) ^ !!(gpio_state & ENCODER_PIN_RIGHT_B_MASK)) ? 1 : -1;
        coder_right += is_pin_a * are_pins_different;
    }
}

static inline void _setup_pin(uint8_t pin)
{
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_set_irq_enabled(pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

void attach_encoder(void)
{
    _setup_pin(ENCODER_PIN_LEFT_A);
    _setup_pin(ENCODER_PIN_LEFT_B);
    _setup_pin(ENCODER_PIN_RIGHT_A);
    _setup_pin(ENCODER_PIN_RIGHT_B);
    irq_set_enabled(IO_IRQ_BANK0, true);
    gpio_set_irq_callback(interrupt_callback);
}

