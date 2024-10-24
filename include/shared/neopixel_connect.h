// https://raw.githubusercontent.com/MrYsLab/NeoPixelConnect/refs/heads/master/src/NeoPixelConnect.h
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

#ifndef NEOPIXEL_CONNECT_NEOPIXELCONNECT_H
#define NEOPIXEL_CONNECT_NEOPIXELCONNECT_H

#include <stdlib.h>

#include <hardware/pio.h>
#include <hardware/dma.h>
#include <hardware/clocks.h>
#include <hardware/gpio.h>

#include <ws2812.pio.h>

#ifndef MAXIMUM_NUM_NEOPIXELS
#   define MAXIMUM_NUM_NEOPIXELS 1024
#endif

// Pixel buffer array offsets
#define RED 0
#define GREEN 1
#define BLUE 2


class NeoPixelConnect
{
public:
    /// @brief Constructor
    /// @param pinNumber: GPIO pin that controls the NeoPixel string.
    /// @param numberOfPixels: Number of pixels in the string
    /// @param pio: pio selected - default = pio0. pio1 may be specified
    /// @param sm: state machine selected. Default = 0
    NeoPixelConnect(uint8_t pinNumber, uint16_t numberOfPixels);

    /// @brief Constructor
    /// @param pinNumber: GPIO pin that controls the NeoPixel string.
    /// @param numberOfPixels: Number of pixels in the string
    /// This constructor sets pio=pio0 and sm to 0
    NeoPixelConnect(uint8_t pinNumber, uint16_t numberOfPixels, PIO pio, uint sm);

    /// @brief Destructor
    virtual ~NeoPixelConnect(){};

    ///@brief Initialize the class instance after calling constructor
    /// @param pinNumber: GPIO pin that controls the NeoPixel string.
    /// @param numberOfPixels: Number of pixels in the string
    void init(uint8_t pinNumber, uint16_t numberOfPixels);

    /// @brief Set a NeoPixel to a given color. By setting autoShow to true, change is
    /// displayed immediately.
    /// @param pixelNumber: set a color for a specific neopixel in the string
    /// @param r: red value (0-255)
    /// @param g: green value(0-255)
    /// @param b: blue value (0-255)
    /// @param autoShow: If true, show the change immediately.
    void setPixel(uint16_t pixel_number, uint8_t r=0, uint8_t g=0, uint8_t b=0, bool autoShow=false);
    void setPixel(uint16_t pixel_number, uint32_t rgb=0, bool autoShow=false) {
        setPixel(pixel_number, (rgb>>16) & 0xFF, (rgb >> 8) & 0xFF, rgb & 0xFF, autoShow);
    }

    /// @brief Set all the pixels to "off".
    /// @param autoShow: If true, show the change immediately
    // set all pixels to 0
    void clear(bool autoShow=true);

    /// @brief Fill all the pixels with same value
    /// @param r: red value (0-255)
    /// @param g: green value(0-255)
    /// @param b: blue value (0-255)
    /// @param autoShow: If true, show the change immediately.
    void fill(uint8_t r=0, uint8_t g=0, uint8_t b=0, bool autoShow=true);
    void fill(uint32_t rgb=0, bool autoShow=false) {
        fill((rgb>>16) & 0xFF, (rgb >> 8) & 0xFF, rgb & 0xFF, autoShow);
    }

    /// @brief Display all the pixels in the buffer
    void show(void);

    uint16_t size(void);

    static uint32_t ColorHSV(uint16_t hue, uint8_t sat = 255, uint8_t val = 255) {

      uint8_t r, g, b;

      // Remap 0-65535 to 0-1529. Pure red is CENTERED on the 64K rollover;
      // 0 is not the start of pure red, but the midpoint...a few values above
      // zero and a few below 65536 all yield pure red (similarly, 32768 is the
      // midpoint, not start, of pure cyan). The 8-bit RGB hexcone (256 values
      // each for red, green, blue) really only allows for 1530 distinct hues
      // (not 1536, more on that below), but the full unsigned 16-bit type was
      // chosen for hue so that one's code can easily handle a contiguous color
      // wheel by allowing hue to roll over in either direction.
      hue = (hue * 1530L + 32768) / 65536;
      // Because red is centered on the rollover point (the +32768 above,
      // essentially a fixed-point +0.5), the above actually yields 0 to 1530,
      // where 0 and 1530 would yield the same thing. Rather than apply a
      // costly modulo operator, 1530 is handled as a special case below.

      // So you'd think that the color "hexcone" (the thing that ramps from
      // pure red, to pure yellow, to pure green and so forth back to red,
      // yielding six slices), and with each color component having 256
      // possible values (0-255), might have 1536 possible items (6*256),
      // but in reality there's 1530. This is because the last element in
      // each 256-element slice is equal to the first element of the next
      // slice, and keeping those in there this would create small
      // discontinuities in the color wheel. So the last element of each
      // slice is dropped...we regard only elements 0-254, with item 255
      // being picked up as element 0 of the next slice. Like this:
      // Red to not-quite-pure-yellow is:        255,   0, 0 to 255, 254,   0
      // Pure yellow to not-quite-pure-green is: 255, 255, 0 to   1, 255,   0
      // Pure green to not-quite-pure-cyan is:     0, 255, 0 to   0, 255, 254
      // and so forth. Hence, 1530 distinct hues (0 to 1529), and hence why
      // the constants below are not the multiples of 256 you might expect.

      // Convert hue to R,G,B (nested ifs faster than divide+mod+switch):
      if (hue < 510) { // Red to Green-1
        b = 0;
        if (hue < 255) { //   Red to Yellow-1
          r = 255;
          g = hue;       //     g = 0 to 254
        } else {         //   Yellow to Green-1
          r = 510 - hue; //     r = 255 to 1
          g = 255;
        }
      } else if (hue < 1020) { // Green to Blue-1
        r = 0;
        if (hue < 765) { //   Green to Cyan-1
          g = 255;
          b = hue - 510;  //     b = 0 to 254
        } else {          //   Cyan to Blue-1
          g = 1020 - hue; //     g = 255 to 1
          b = 255;
        }
      } else if (hue < 1530) { // Blue to Red-1
        g = 0;
        if (hue < 1275) { //   Blue to Magenta-1
          r = hue - 1020; //     r = 0 to 254
          b = 255;
        } else { //   Magenta to Red-1
          r = 255;
          b = 1530 - hue; //     b = 255 to 1
        }
      } else { // Last 0.5 Red (quicker than % operator)
        r = 255;
        g = b = 0;
      }

      // Apply saturation and value to R,G,B, pack into 32-bit result:
      uint32_t v1 = 1 + val;  // 1 to 256; allows >>8 instead of /255
      uint16_t s1 = 1 + sat;  // 1 to 256; same reason
      uint8_t s2 = 255 - sat; // 255 to 0
      return ((((((r * s1) >> 8) + s2) * v1) & 0xff00) << 8) |
             (((((g * s1) >> 8) + s2) * v1) & 0xff00) |
             (((((b * s1) >> 8) + s2) * v1) >> 8);
    }


private:
    /// @brief set a pixel's value to reflect pixel_grb
    /// @param pixel_grb: rgb represented as a 32 bit value
    void putPixel(uint32_t pixel_grb); //{

    // pio - 0 or 1
    PIO pixelPio;

    // calculated program offset in memory
    uint pixelOffset;

    // pio state machine to use
    uint pixelSm;

    // number of pixels in the strip
    uint16_t actual_number_of_pixels;

    // a buffer that holds the color for each pixel
    uint8_t pixelBuffer[MAXIMUM_NUM_NEOPIXELS][3];

    // create a 32 bit value combining the 3 colors
    uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b);
};


#endif //NEOPIXEL_CONNECT_NEOPIXELCONNECT_H
