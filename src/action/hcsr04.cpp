#include <action/hcsr04.hpp>

#include <hardware/gpio.h>

#define IRQ_MASK (GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE)

HCSR04* HCSR04::irqHandles[PICO_PIN_COUNT] = {nullptr};

HCSR04::HCSR04(uint8_t trig, uint8_t echo) {
	this->trig = trig;
	this->echo = echo;
	this->lastDist = 0;
	this->newValue = false;

	gpio_init(this->trig);
	gpio_init(this->echo);
	gpio_set_dir(this->trig, GPIO_OUT);
	gpio_set_dir(this->echo, GPIO_IN);

	HCSR04::irqHandles[this->echo] = this;

	gpio_set_irq_enabled(this->echo, IRQ_MASK, true);
	irq_set_enabled(IO_IRQ_BANK0, true);
	gpio_add_raw_irq_handler(this->echo, HCSR04::irqHandler);
}

HCSR04::~HCSR04() {
	HCSR04::irqHandles[this->echo] = this;
	gpio_deinit(this->echo);
	gpio_deinit(this->trig);
	gpio_set_irq_enabled(this->echo, IRQ_MASK, false);
	gpio_remove_raw_irq_handler(this->echo, HCSR04::irqHandler);
}

void HCSR04::irqHandler() {
	uint64_t irq_time = to_us_since_boot(get_absolute_time());
	HCSR04 *inst = nullptr;
	uint32_t irq_mask = 0;
	for (uint8_t i=0;i<PICO_PIN_COUNT;i++) {
		inst = HCSR04::irqHandles[i];
		if (!inst)
			continue;
		irq_mask = gpio_get_irq_event_mask(inst->echo);
		if (!(irq_mask & IRQ_MASK))
			continue;
		gpio_acknowledge_irq(inst->echo, IRQ_MASK);
		break;
	}
	if (!inst)
		return;
	if (irq_mask & GPIO_IRQ_EDGE_RISE) {
		inst->riseTime = irq_time;
	} else if (irq_mask & GPIO_IRQ_EDGE_FALL) {
		float dst = ((float)(irq_time-inst->riseTime))*(1e-6*343.0f/2.0f)*1e3;
		if (dst > 1000.0f)
			return;
		inst->lastDist = dst;
		inst->newValue = true;
	}
}

bool HCSR04::hasNewDist() {
	return this->newValue;
}

// Non blocking
void HCSR04::trigger() {
	gpio_put(this->trig, true);
	busy_wait_us(10);
	gpio_put(this->trig, false);
}

float HCSR04::getLastDistance() {
	this->newValue = false;
	return this->lastDist;
}

// Blocking
float HCSR04::getDistance() {
	if (this->newValue)
		return this->lastDist;
	this->trigger();
	busy_wait_us(60000);
	return getLastDistance();
}