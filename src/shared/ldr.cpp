#include <shared/ldr.hpp>
#include <hardware/adc.h>
#include <cmath>

#define PICO_FIRST_ADC_PIN 26

LDR::LDR(uint8_t pin, float r_base, float coeff, float exponent) : pin(pin), r_base(r_base), coeff(coeff), exponent(exponent) {
	adc_init();

	adc_gpio_init(pin);
	adc_select_input(pin - PICO_FIRST_ADC_PIN);
}

LDR::~LDR() {

}

float LDR::readLux() {
	const float conversion_factor = 3.3f / (1 << 12);
	adc_select_input(this->pin - PICO_FIRST_ADC_PIN);
	float volRes = adc_read() * conversion_factor;
	float volPhoto = 3.3f - volRes;
	float resPhoto = volPhoto / volRes * this->r_base;
	return this->coeff * std::powf(resPhoto, this->exponent);
}