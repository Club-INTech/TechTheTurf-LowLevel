#pragma once

#include <cmath>
#include <cstdint>

static inline float calculateLipoPercentage(float voltage, int series = 0) {
	// Guess number of S
	if (series == 0) {
		series = 1;
		while (voltage/series > 4.4)
			series++;
	}
	float perCell = voltage/series;
	// https://electronics.stackexchange.com/questions/435837/calculate-battery-percentage-on-lipo-battery
	return 123.0f - (123.0f/std::pow(1.0f+std::pow(perCell/3.7f, 80.0f), 0.165f));
}

static inline uint32_t colorLerp(uint32_t from, uint32_t to, float perc) {
	uint8_t r = ((from >> 16)&0xFF)*(1.0f-perc) + ((to >> 16)&0xFF)*perc;
	uint8_t g = ((from >> 8)&0xFF)*(1.0f-perc) + ((to >> 8)&0xFF)*perc;
	uint8_t b = ((from >> 0)&0xFF)*(1.0f-perc) + ((to >> 0)&0xFF)*perc;
	return r << 16 | g << 8 | b;
}