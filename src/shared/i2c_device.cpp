#include <string.h>
#include <shared/i2c_device.hpp>
#include <hardware/gpio.h>

I2CDevice::I2CDevice(i2c_inst_t *inst, uint8_t sda, uint8_t scl, uint8_t addr, uint baudrate, bool pullUp, bool swapEndian, uint timeoutUs)
 : sda(sda), scl(scl), addr(addr), timeout(timeoutUs), swapEndian(swapEndian), inst(inst) {
	i2c_init(inst, baudrate);

	gpio_set_function(sda, GPIO_FUNC_I2C);
	gpio_set_function(scl, GPIO_FUNC_I2C);

	gpio_set_pulls(sda, pullUp, false);
	gpio_set_pulls(scl, pullUp, false);
}

I2CDevice::~I2CDevice() {
	i2c_deinit(this->inst);

	gpio_set_function(this->sda, GPIO_FUNC_NULL);
	gpio_set_function(this->scl, GPIO_FUNC_NULL);

	gpio_set_pulls(this->sda, false, false);
	gpio_set_pulls(this->scl, false, false);
}

int I2CDevice::writeRaw(const uint8_t *data, size_t size, bool nostop) {
	return i2c_write_timeout_us(this->inst, this->addr, data, size, nostop, this->timeout);
}

int I2CDevice::readRaw(uint8_t *data, size_t size, bool nostop) {
	return i2c_read_timeout_us(this->inst, this->addr, data, size, nostop, this->timeout);
}

int I2CDevice::write(uint8_t reg, const uint8_t *data, size_t size) {
	this->buffer[0] = reg;
	if (size > I2CDEVICE_MAX_I2C_SIZE)
		size = I2CDEVICE_MAX_I2C_SIZE;
	memcpy(&buffer[1], data, size);
	return writeRaw(this->buffer, size+1);
}

int I2CDevice::read(uint8_t reg, uint8_t *data, size_t size) {
	int ret = writeRaw(&reg, sizeof(uint8_t), true);
	if (ret != sizeof(uint8_t))
		return ret;
	return readRaw(data, size);	
}

int I2CDevice::writeU8(uint8_t reg, uint8_t data) {
	uint8_t buffer[1+1] = {reg, data};
	return writeRaw(buffer, sizeof(buffer));
}

int I2CDevice::writeU16(uint8_t reg, uint16_t data) {
	uint8_t buffer[1+2] = {reg, 0, 0};
	if (this->swapEndian)
		data = __builtin_bswap16(data);
	memcpy(&buffer[1], &data, sizeof(data));
	return writeRaw(buffer, sizeof(buffer));
}
int I2CDevice::writeU32(uint8_t reg, uint32_t data) {
	uint8_t buffer[1+4] = {reg, 0, 0};
	if (this->swapEndian)
		data = __builtin_bswap32(data);
	memcpy(&buffer[1], &data, sizeof(data));
	return writeRaw(buffer, sizeof(buffer));
}


int I2CDevice::readU8(uint8_t reg, uint8_t *data) {
	return read(reg, data, sizeof(uint8_t));
}

int I2CDevice::readU16(uint8_t reg, uint16_t *data) {
	int ret = read(reg, (uint8_t*)data, sizeof(uint16_t));
	if (this->swapEndian)
		*data =  __builtin_bswap16(*data);
	return ret;
}

int I2CDevice::readU32(uint8_t reg, uint32_t *data) {
	int ret = read(reg, (uint8_t*)data, sizeof(uint32_t));
	if (this->swapEndian)
		*data =  __builtin_bswap32(*data);
	return ret;
}