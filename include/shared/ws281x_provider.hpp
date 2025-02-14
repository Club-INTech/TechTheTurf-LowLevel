#pragma once

#include <shared/led_provider.hpp>

struct LedParams {
	LedFunction func;
	LedPosition pos;
	bool invert_rg;
};

class WS281XProvider : public LedProvider {
public:
	WS281XProvider(uint8_t ws_pin, uint32_t count, PIO pio=pio1, uint8_t sm=0);
	~WS281XProvider();
	uint32_t getSize();

	void setColorRaw(uint32_t idx, uint32_t rgb, uint8_t brightness);

	void setLedOrder(uint32_t idx, bool brg);
	void setLedOrderRange(uint32_t from, uint32_t to, bool brg);

	void setLedParams(uint32_t idx, LedFunction func, LedPosition pos);

	LedPosition getLedPosition(uint32_t idx);
	LedFunction getLedFunction(uint32_t idx);

	void clear();
	void display();

private:
	// Stolen from NeoPixel: https://github.com/adafruit/Adafruit_NeoPixel/blob/aa798ff5e9bb9d7299190627d2454ccd6f599ae8/Adafruit_NeoPixel.h#L183
	// Python code to generate:
	// import math;gamma=2.6;dat=["{:3}".format(round(math.pow(float(x)/255.0,gamma)*255.0)) for x in range(256)];
	// print("static constexpr uint8_t gammaTable[256] = {\n"+",\n".join([",".join(dat[0+i:15+i]) for i in range(0,len(dat),15)])+"};")
	static constexpr uint8_t gammaTable[256] = {
		  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
		  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  1,
		  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,  2,  3,
		  3,  3,  3,  3,  3,  4,  4,  4,  4,  5,  5,  5,  5,  5,  6,
		  6,  6,  6,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10, 10, 10,
		 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16, 17,
		 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
		 25, 26, 27, 27, 28, 29, 29, 30, 31, 31, 32, 33, 34, 34, 35,
		 36, 37, 38, 38, 39, 40, 41, 42, 42, 43, 44, 45, 46, 47, 48,
		 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63,
		 64, 65, 66, 68, 69, 70, 71, 72, 73, 75, 76, 77, 78, 80, 81,
		 82, 84, 85, 86, 88, 89, 90, 92, 93, 94, 96, 97, 99,100,102,
		103,105,106,108,109,111,112,114,115,117,119,120,122,124,125,
		127,129,130,132,134,136,137,139,141,143,145,146,148,150,152,
		154,156,158,160,162,164,166,168,170,172,174,176,178,180,182,
		184,186,188,191,193,195,197,199,202,204,206,209,211,213,215,
		218,220,223,225,227,230,232,235,237,240,242,245,247,250,252,
		255};

	uint32_t size;
	NeoPixelConnect pixels;
	LedParams* params;
};
