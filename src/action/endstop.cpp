#include <action/endstop.hpp>

// Todo: convert to IRQ

Endstop::Endstop(uint8_t pin) {
	this->pin = pin;

	gpio_init(pin);

	gpio_set_dir(pin, false);

	this->state = !gpio_get(pin);
}

Endstop::~Endstop() {
	gpio_deinit(this->pin);
}

bool Endstop::poll() {
	this->state = !gpio_get(pin);
	return this->state;
}
