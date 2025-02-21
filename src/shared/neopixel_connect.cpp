//https://raw.githubusercontent.com/MrYsLab/NeoPixelConnect/refs/heads/master/src/NeoPixelConnect.cpp
/*
 Copyright (c) 2020-2022 Alan Yorinks All rights reserved.

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU AFFERO GENERAL PUBLIC LICENSE
 Version 3 as published by the Free Software Foundation; either
 or (at your option) any later version.
 This library is distributed in the hope that it will be useful,f
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 General Public License for more details.

 You should have received a copy of the GNU AFFERO GENERAL PUBLIC LICENSE
 along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */
#include <cstdint>
#include <hardware/pio.h>
#include <hardware/regs/dreq.h>
#include <hardware/timer.h>
#include <shared/neopixel_connect.h>
#include <hardware/clocks.h>
#include <hardware/dma.h>
#include <string.h>
#include <cstdio>

/// @brief Constructor
/// @param pinNumber: GPIO pin that controls the NeoPixel string.
/// @param numberOfPixels: Number of pixels in the string
/// @param pio: pio selected - default = pio0. pio1 may be specified
/// @param sm: state machine selected. Default = 0
NeoPixelConnect::NeoPixelConnect(uint8_t pinNumber, uint16_t numberOfPixels, PIO pio, uint sm) {
	this->pixelSm = sm;
	this->pixelPio = pio;
	this->init(pinNumber, numberOfPixels);
}

NeoPixelConnect::~NeoPixelConnect() {
	free(this->pixelBuffer);
}

/// @brief Continuation of Constructor
/// @param pinNumber: GPIO pin that controls the NeoPixel string.
/// @param numberOfPixels: Number of pixels in the string
void NeoPixelConnect::init(uint8_t pinNumber, uint16_t numberOfPixels) {
	this->pixelBuffer = (uint8_t*)aligned_alloc(4, numberOfPixels * 4 * sizeof(uint8_t));
	// save the number of pixels in use
	this->realPixelCnt = numberOfPixels;

	uint offset = pio_add_program(this->pixelPio, &ws2812_program);
	programInit(this->pixelPio, this->pixelSm, offset, pinNumber, 800000, false);

	// Configure a channel to write the same word (32 bits) repeatedly to PIO0
	// SM0's TX FIFO, paced by the data request signal from that peripheral.
	this->dmaChannel = dma_claim_unused_channel(true);
	dma_channel_config dmaConfig = dma_channel_get_default_config(this->dmaChannel);
	channel_config_set_transfer_data_size(&dmaConfig, DMA_SIZE_32);
	channel_config_set_read_increment(&dmaConfig, true);
	channel_config_set_irq_quiet(&dmaConfig, true);
	channel_config_set_dreq(&dmaConfig, pio_get_dreq(this->pixelPio, this->pixelSm, true));

	dma_channel_configure(
		this->dmaChannel,
		&dmaConfig,
		&this->pixelPio->txf[this->pixelSm], // Write address (only need to set this once)
		nullptr,             // Don't provide a read address yet
		numberOfPixels, // Number of transfers
		false             // Don't start yet
	);

	// clear & display
	clear(true);
}

/// @brief Set a NeoPixel to a given color. By setting autoShow to true, change is
/// displayed immediately.
/// @param pixelNumber: set a color for a specific neopixel in the string
/// @param r: red value (0-255)
/// @param g: green value(0-255)
/// @param b: blue value (0-255)
/// @param autoShow: If true, show the change immediately.
void NeoPixelConnect::setPixel(uint16_t pixelNumber, uint8_t r, uint8_t g, uint8_t b, bool autoShow) {
	this->pixelBuffer[(pixelNumber*4) + RED] = r;
	this->pixelBuffer[(pixelNumber*4) + GREEN] = g;
	this->pixelBuffer[(pixelNumber*4) + BLUE] = b;

	if (autoShow)
		this->show();
}

/// @brief Set all the pixels to "off".
/// @param autoShow: If true, show the change immediately
void NeoPixelConnect::clear(bool autoShow) {
	// set all the neopixels in the buffer to all zeroes

	memset(this->pixelBuffer, 0, this->realPixelCnt*4*sizeof(uint8_t));

	if (autoShow)
		this->show();
}

/// @brief Fill all the pixels with same value
/// @param r: red value (0-255)
/// @param g: green value(0-255)
/// @param b: blue value (0-255)
/// @param autoShow: If true, show the change immediately.
void NeoPixelConnect::fill(uint8_t r, uint8_t g, uint8_t b, bool autoShow) {
	// fill all the neopixels in the buffer with the
	// specified rgb values.
	for (uint16_t i = 0; i < this->realPixelCnt; i++) {
		this->pixelBuffer[(i*4) + RED] = r;
		this->pixelBuffer[(i*4) + GREEN] = g;
		this->pixelBuffer[(i*4) + BLUE] = b;
	}
	if (autoShow)
		this->show();
}

/// @brief Display all the pixels in the buffer
void NeoPixelConnect::show(void) {
	// Launch DMA transfer
	//dma_channel_wait_for_finish_blocking(this->dmaChannel);
	dma_channel_set_read_addr(this->dmaChannel, this->pixelBuffer, true);
}

uint16_t NeoPixelConnect::size(void) {
	return this->realPixelCnt;
}

void NeoPixelConnect::programInit(PIO pio, uint sm, uint offset, uint pin, float freq, bool rgbw) {
	pio_gpio_init(pio, pin);
	pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);

	pio_sm_config c = ws2812_program_get_default_config(offset);
	sm_config_set_sideset_pins(&c, pin);
	sm_config_set_out_shift(&c, false, true, rgbw ? 32 : 24);
	sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

	int cycles_per_bit = ws2812_T1 + ws2812_T2 + ws2812_T3;
	float div = clock_get_hz(clk_sys) / (freq * cycles_per_bit);
	sm_config_set_clkdiv(&c, div);

	pio_sm_init(pio, sm, offset, &c);
	pio_sm_set_enabled(pio, sm, true);
}