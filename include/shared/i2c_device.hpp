#pragma once

#include <cstdint>
#include <hardware/i2c.h>

#define I2CDEVICE_MAX_I2C_SIZE 1024

class I2CDevice
{
public:
	I2CDevice(i2c_inst_t *inst, uint8_t sda, uint8_t scl, uint8_t addr, uint baudrate=100e3, bool pullUp=true, bool swapEndian=false, uint timeoutUs=200);
	~I2CDevice();

	void setTimeout(uint timeoutUs);
	uint getTimeout();

	int writeU8(uint8_t reg, uint8_t data);
	int writeU16(uint8_t reg, uint16_t data);
	int writeU32(uint8_t reg, uint32_t data);
	int write(uint8_t reg, const uint8_t *data, size_t size);
	int writeRaw(const uint8_t *data, size_t size, bool nostop = false);

	int readU8(uint8_t reg, uint8_t *data);
	int readU16(uint8_t reg, uint16_t *data);
	int readU32(uint8_t reg, uint32_t *data);
	int read(uint8_t reg, uint8_t *data, size_t size);
	int readRaw(uint8_t *data, size_t size, bool nostop = false);

private:
	uint8_t sda, scl;
	uint8_t addr;
	uint timeout;
	bool swapEndian;
	i2c_inst_t *inst;

	uint8_t buffer[I2CDEVICE_MAX_I2C_SIZE+1];
};