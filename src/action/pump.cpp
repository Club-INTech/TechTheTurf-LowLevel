#include <action/pump.hpp>

Pump::Pump(uint8_t pin) {
	this->pin = pin;
	gpio_init(pin);
	gpio_set_dir(pin, true);
	this->setEnable(false);
}

Pump::~Pump() {
	gpio_deinit(this->pin);
}

bool Pump::getState() {
	return this->state;
}

void Pump::setEnable(bool en) {
	this->state = en;
	gpio_put(this->pin, en);
}

void Pump::enable() {
	this->setEnable(true);
}

void Pump::disable() {
	this->setEnable(false);
}