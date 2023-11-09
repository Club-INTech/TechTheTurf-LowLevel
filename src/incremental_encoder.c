#include "incremental_encoder.h"

#include "hardware/gpio.h"
#include "hardware/irq.h"
#include <stdio.h>

#define ENCODER_PIN_LEFT_A      17
#define ENCODER_PIN_LEFT_B      16
#define ENCODER_PIN_RIGHT_A     19
#define ENCODER_PIN_RIGHT_B     18

#define ENCODER_PIN_LEFT_A_MASK         (1 << (ENCODER_PIN_LEFT_A))
#define ENCODER_PIN_LEFT_B_MASK         (1 << (ENCODER_PIN_LEFT_B))
#define ENCODER_PIN_RIGHT_A_MASK        (1 << (ENCODER_PIN_RIGHT_A))
#define ENCODER_PIN_RIGHT_B_MASK        (1 << (ENCODER_PIN_RIGHT_B))

volatile static int32_t coder_left = 0;
volatile static int32_t coder_right = 0;

int32_t get_coder_left(void)
{
    return coder_left;
}

int32_t get_coder_right(void)
{
    return coder_right;
}

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
void interrupt_callback(uint gpio, uint32_t event_mask)
{
    // If we have more interrupts in the future, best to check but here it
    // should be fiiiine the callback only gets called on rising and falling
    // edges anyways
    // if (event_mask != GPIO_IRQ_EDGE_RISE && event_mask != GPIO_IRQ_EDGE_FALL)
    //     return;

    uint32_t gpio_state = gpio_get_all();
    if (gpio == ENCODER_PIN_LEFT_A || gpio == ENCODER_PIN_LEFT_B)
    {
        bool is_pin_a = gpio == ENCODER_PIN_LEFT_A;
        // a ^ b <=> !a ^ !b, and a & b does not guarantee the result will
        // be 0 or 1, so if we take the negation of each statement, we get the
        // ^ working without the need of any !! statements
        bool are_pins_different = !(gpio_state & ENCODER_PIN_LEFT_A_MASK) ^ !(gpio_state & ENCODER_PIN_LEFT_B_MASK);
        coder_left += (is_pin_a == are_pins_different) ? 1 : -1;
    }
    else if (gpio == ENCODER_PIN_RIGHT_A || gpio == ENCODER_PIN_RIGHT_B)
    {
        bool is_pin_a = gpio == ENCODER_PIN_RIGHT_A;
        // same as in the other if
        bool are_pins_different = !(gpio_state & ENCODER_PIN_RIGHT_A_MASK) ^ !(gpio_state & ENCODER_PIN_RIGHT_B_MASK);
        coder_right += (is_pin_a == are_pins_different) ? 1 : -1;
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

