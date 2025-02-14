#pragma once

#include <functional>
#include <shared/i2c_device.hpp>
#include <Invn/Drivers/Icm426xx/Icm426xxDriver_HL.h>

#define INV_LOGLEVEL INV_MSG_LEVEL_DEBUG

class ICM42688 : public I2CDevice
{
public:
	ICM42688(i2c_inst_t *inst, uint8_t sda, uint8_t scl, uint8_t addr, uint8_t int1_pin, uint8_t fsync_pin, uint baudrate=100e3, bool pullUp=true, uint timeoutUs=1000);

	int setGyroFsr(ICM426XX_GYRO_CONFIG0_FS_SEL_t fsr);
	int setAccelFsr(ICM426XX_ACCEL_CONFIG0_FS_SEL_t fsr);
	int setGyroRate(ICM426XX_GYRO_CONFIG0_ODR_t rate);
	int setAccelRate(ICM426XX_ACCEL_CONFIG0_ODR_t rate);

	int setFsync(bool enable);
	int setFifo(bool enable);
	int setHiResFifo(bool enable);
	int setFifoWm(uint16_t watermark);
	int setTimestampRegister(bool enable);

	int setGyro(bool enable);
	int setAccel(bool enable, bool lowNoise=true);

	int resetFifo();

	int setIbi(inv_icm426xx_interrupt_parameter_t *config);
	int setInt1(inv_icm426xx_interrupt_parameter_t *config);
	int setInt2(inv_icm426xx_interrupt_parameter_t *config);

	int getIbi(inv_icm426xx_interrupt_parameter_t *config);
	int getInt1(inv_icm426xx_interrupt_parameter_t *config);
	int getInt2(inv_icm426xx_interrupt_parameter_t *config);

	void setIntCallback(std::function<int(ICM42688*)> func);

	int readValues();
	int selfTest();

	void sensorCallback(inv_icm426xx_sensor_event_t *event);
	void intCallback();
private:

	int setup();
	void setupIRQ();

	struct inv_icm426xx driver;
	struct inv_icm426xx_serif serif;
	uint8_t int1,fsync;
	bool fifo, hiFifo;
	std::function<int(ICM42688*)> cbfunc;
};