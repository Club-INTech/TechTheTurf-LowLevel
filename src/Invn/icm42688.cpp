#include "Invn/Drivers/Icm426xx/Icm426xxDriver_HL.h"
#include "Invn/EmbUtils/ErrorHelper.h"
#include <Invn/EmbUtils/Message.h>
#include <Invn/icm42688.hpp>
#include <Invn/Drivers/Icm426xx/Icm426xxSelfTest.h>
#include <cstdarg>
#include <cstdint>
#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <stdio.h>
#include <hardware/timer.h>
#include <pico/time.h>

/*
 * Icm426xx driver needs to get time in us. Let's give its implementation here.
 */
extern "C" uint64_t inv_icm426xx_get_time_us(void) {
	return to_us_since_boot(get_absolute_time());
}

/*
 * Icm426xx driver needs a sleep feature from external device. Thus inv_icm426xx_sleep_us
 * is defined as extern symbol in driver. Let's give its implementation here.
 */
extern "C" void inv_icm426xx_sleep_us(uint32_t us) {
	busy_wait_us(us);
}

/*
 * Printer function for message facility
 */
void inv_msg_printer(int level, const char *str, va_list ap)
{
	static char out_str[256]; /* static to limit stack usage */
	unsigned    idx                  = 0;
	const char *s[6] = {
		"", // INV_MSG_LEVEL_OFF
		"[E] ", // INV_MSG_LEVEL_ERROR
		"[W] ", // INV_MSG_LEVEL_WARNING
		"[I] ", // INV_MSG_LEVEL_INFO
		"[V] ", // INV_MSG_LEVEL_VERBOSE
		"[D] ", // INV_MSG_LEVEL_DEBUG
	};
	idx += snprintf(&out_str[idx], sizeof(out_str) - idx, "%s", s[level]);
	if (idx >= (sizeof(out_str)))
		return;
	idx += vsnprintf(&out_str[idx], sizeof(out_str) - idx, str, ap);
	if (idx >= (sizeof(out_str)))
		return;
	idx += snprintf(&out_str[idx], sizeof(out_str) - idx, "\n");
	if (idx >= (sizeof(out_str)))
		return;

	out_str[idx] = '\x00';

	printf(out_str);
}

int inv_io_hal_read_reg(struct inv_icm426xx_serif *serif, uint8_t reg, uint8_t *buf, uint32_t len) {
	int ret = ((ICM42688*)(serif->context))->read(reg, buf, len);
	return ret > 0 ? 0 : ret;
}

int inv_io_hal_write_reg(struct inv_icm426xx_serif *serif, uint8_t reg, const uint8_t *buf, uint32_t len) {
	int ret = ((ICM42688*)(serif->context))->write(reg, buf, len);
	return ret > 0 ? 0 : ret;
}

void inv_sensor_callback(inv_icm426xx_sensor_event_t *event, void *context) {
	((ICM42688*)context)->sensorCallback(event);
}

struct irq_data {
	uint8_t pin;
	ICM42688 *inst;
};

irq_data icm_irq_table[NUM_BANK0_GPIOS] = {{255,nullptr}};
uint8_t icm_irq_table_idx = 0;

int ICMAddIrq(uint8_t pin, ICM42688 *inst) {
	if (icm_irq_table_idx >= NUM_BANK0_GPIOS)
		return -1;
	icm_irq_table[icm_irq_table_idx++] = {pin, inst};
	return 0;
}

void ICMRemoveIrq(uint8_t pin) {
	for (int i=0;i<icm_irq_table_idx;i++) {
		if (icm_irq_table[i].pin != pin)
			continue;
		icm_irq_table[i] = icm_irq_table[icm_irq_table_idx-1];
		icm_irq_table[icm_irq_table_idx-1] = {255, nullptr};
		icm_irq_table_idx--;
		return;
	}
}

void ICMIntCb() {
	printf("irq\n");
	uint8_t pin = 255;
	ICM42688 *inst = nullptr;
	for (int i=0;i<icm_irq_table_idx;i++) {
		if (gpio_get_irq_event_mask(icm_irq_table[i].pin) & GPIO_IRQ_EDGE_FALL) {
			pin = icm_irq_table[i].pin;
			inst = icm_irq_table[i].inst;
			break;
		}
	}
	if (!inst)
		return;
	gpio_acknowledge_irq(pin, GPIO_IRQ_EDGE_FALL);
	inst->intCallback();
}

ICM42688::ICM42688(i2c_inst_t *inst, uint8_t sda, uint8_t scl, uint8_t addr, uint8_t int1_pin, uint8_t fsync_pin, uint baudrate, bool pullUp, uint timeoutUs)
 : I2CDevice(inst, sda, scl, addr, baudrate, pullUp, true, timeoutUs), int1(int1_pin), fsync(fsync_pin), fifo(false), hiFifo(false) {
 	INV_MSG_SETUP(INV_LOGLEVEL, inv_msg_printer);

 	this->serif.context    = this;
	this->serif.read_reg   = inv_io_hal_read_reg;
	this->serif.write_reg  = inv_io_hal_write_reg;
	this->serif.max_read   = I2CDEVICE_MAX_I2C_SIZE; // maximum number of bytes allowed per serial read
	this->serif.max_write  = I2CDEVICE_MAX_I2C_SIZE; // maximum number of bytes allowed per serial write
	this->serif.serif_type = ICM426XX_UI_I2C;

	setup();
	setupIRQ();
}

int ICM42688::setup() {
	INV_MSG(INV_MSG_LEVEL_VERBOSE, "Initialize ICM");

	/*typedef void (*sensor_event_cb)(inv_icm426xx_sensor_event_t *event);

	auto sensor_cb = [this](inv_icm426xx_sensor_event_t *event) {
		return this->sensorCallback(event);
	};*/

	/* Init device sensor_cb.target<sensor_event_cb>()*/
	int rc = inv_icm426xx_init(&this->driver, &this->serif, inv_sensor_callback);
	if (rc != INV_ERROR_SUCCESS) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "!!! ERROR : failed to initialize Icm426xx. %s", inv_error_str(rc));
		return rc;
	}

	/* Check WHOAMI */
	INV_MSG(INV_MSG_LEVEL_VERBOSE, "Check ICM whoami value");

	uint8_t who_am_i;
	rc = inv_icm426xx_get_who_am_i(&this->driver, &who_am_i);
	if (rc != INV_ERROR_SUCCESS) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "!!! ERROR : failed to read Icm426xx whoami value. %s", inv_error_str(rc));
		return rc;
	}

	if (who_am_i != ICM_WHOAMI) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "!!! ERROR :  bad WHOAMI value. Got 0x%02x (expected: 0x%02x)",
		        who_am_i, ICM_WHOAMI);
		return INV_ERROR;
	}

	rc |= setAccelFsr(ICM426XX_ACCEL_CONFIG0_FS_SEL_4g);
	rc |= setGyroFsr(ICM426XX_GYRO_CONFIG0_FS_SEL_500dps);
	rc |= setAccelRate(ICM426XX_ACCEL_CONFIG0_ODR_1_KHZ);
	rc |= setGyroRate(ICM426XX_GYRO_CONFIG0_ODR_1_KHZ);

	rc |= inv_icm426xx_enable_accel_low_noise_mode(&this->driver);
	rc |= inv_icm426xx_enable_gyro_low_noise_mode(&this->driver);

	return rc;
}

void ICM42688::setupIRQ() {
	ICMAddIrq(this->int1, this);
	gpio_init(this->int1);
	gpio_set_pulls(this->int1, false, false);
	gpio_set_irq_enabled(this->int1, GPIO_IRQ_EDGE_FALL, true);
	irq_set_enabled(IO_IRQ_BANK0, true);
	gpio_add_raw_irq_handler(this->int1, ICMIntCb);
}

void ICM42688::setIntCallback(std::function<int(ICM42688*)> func) {
	this->cbfunc = func;
}

void ICM42688::intCallback() {
	if (this->cbfunc && this->cbfunc(this) == 0)
		return;

	if (this->fifo) {
		inv_icm426xx_get_data_from_fifo(&this->driver);
	} else {
		inv_icm426xx_get_data_from_registers(&this->driver);
	}
}

void ICM42688::sensorCallback(inv_icm426xx_sensor_event_t *event) {
	bool ACCEL = (event->sensor_mask >> INV_ICM426XX_SENSOR_ACCEL) & 1;
	bool GYRO = (event->sensor_mask >> INV_ICM426XX_SENSOR_GYRO) & 1;
	bool FSYNC_EVENT = (event->sensor_mask >> INV_ICM426XX_SENSOR_FSYNC_EVENT) & 1;
	bool OIS = (event->sensor_mask >> INV_ICM426XX_SENSOR_OIS) & 1;
	bool TEMPERATURE = (event->sensor_mask >> INV_ICM426XX_SENSOR_TEMPERATURE) & 1;
	bool TAP = (event->sensor_mask >> INV_ICM426XX_SENSOR_TAP) & 1;
	bool DMP_PEDOMETER_EVENT = (event->sensor_mask >> INV_ICM426XX_SENSOR_DMP_PEDOMETER_EVENT) & 1;
	bool DMP_PEDOMETER_COUNT = (event->sensor_mask >> INV_ICM426XX_SENSOR_DMP_PEDOMETER_COUNT) & 1;
	bool DMP_TILT = (event->sensor_mask >> INV_ICM426XX_SENSOR_DMP_TILT) & 1;
	bool DMP_R2W = (event->sensor_mask >> INV_ICM426XX_SENSOR_DMP_R2W) & 1;
	printf("a:%i g:%i fs:%i ois:%i t:%i tap:%i dpe:%i dpc:%i dt:%i dr:%i\n", ACCEL,GYRO,FSYNC_EVENT,OIS,TEMPERATURE,TAP,DMP_PEDOMETER_EVENT,DMP_PEDOMETER_COUNT,DMP_TILT,DMP_R2W);
	printf("%d %d %d\n", event->accel[0], event->accel[1], event->accel[2]);
	printf("%d %d %d\n", event->gyro[0], event->gyro[1], event->gyro[2]);
}

int ICM42688::setGyroFsr(ICM426XX_GYRO_CONFIG0_FS_SEL_t fsr) {
	return inv_icm426xx_set_gyro_fsr(&this->driver, fsr);
}
int ICM42688::setGyroRate(ICM426XX_GYRO_CONFIG0_ODR_t rate) {
	return inv_icm426xx_set_gyro_frequency(&this->driver, rate);
}
int ICM42688::setAccelFsr(ICM426XX_ACCEL_CONFIG0_FS_SEL_t fsr) {
	return inv_icm426xx_set_accel_fsr(&this->driver, fsr);
}
int ICM42688::setAccelRate(ICM426XX_ACCEL_CONFIG0_ODR_t rate) {
	return inv_icm426xx_set_accel_frequency(&this->driver, rate);
}

int ICM42688::setFsync(bool enable) {
	if (enable)
		return inv_icm426xx_enable_fsync(&this->driver);

	return inv_icm426xx_disable_fsync(&this->driver);
}

int ICM42688::resetFifo() {
	return inv_icm426xx_reset_fifo(&this->driver);
}

int ICM42688::setGyro(bool enable) {
	if (enable)
		return inv_icm426xx_enable_gyro_low_noise_mode(&this->driver);

	return inv_icm426xx_disable_gyro(&this->driver);
}

int ICM42688::setAccel(bool enable, bool lowNoise) {
	if (!enable)
		return inv_icm426xx_disable_accel(&this->driver);

	if (lowNoise)
		return inv_icm426xx_enable_accel_low_noise_mode(&this->driver);
	else
		return inv_icm426xx_enable_accel_low_power_mode(&this->driver);
}

int ICM42688::setFifo(bool enable) {
	this->fifo = enable;
	return inv_icm426xx_configure_fifo(&this->driver, enable ? INV_ICM426XX_FIFO_ENABLED : INV_ICM426XX_FIFO_DISABLED);
}

int ICM42688::setHiResFifo(bool enable) {
	this->hiFifo = enable;
	if (enable)
		return inv_icm426xx_enable_high_resolution_fifo(&this->driver);

	return inv_icm426xx_disable_high_resolution_fifo(&this->driver);
}

int ICM42688::setFifoWm(uint16_t watermark) {
	return inv_icm426xx_configure_fifo_wm(&this->driver, watermark);
}

int ICM42688::setTimestampRegister(bool enable) {
	if (enable)
		return inv_icm426xx_enable_timestamp_to_register(&this->driver);

	return inv_icm426xx_disable_timestamp_to_register(&this->driver);
}

int ICM42688::setIbi(inv_icm426xx_interrupt_parameter_t *config) {
	return inv_icm426xx_set_config_ibi(&this->driver, config);
}

int ICM42688::setInt1(inv_icm426xx_interrupt_parameter_t *config) {
	return inv_icm426xx_set_config_int1(&this->driver, config);
}

int ICM42688::setInt2(inv_icm426xx_interrupt_parameter_t *config) {
	return inv_icm426xx_set_config_int2(&this->driver, config);
}

int ICM42688::getIbi(inv_icm426xx_interrupt_parameter_t *config) {
	return inv_icm426xx_get_config_ibi(&this->driver, config);
}

int ICM42688::getInt1(inv_icm426xx_interrupt_parameter_t *config) {
	return inv_icm426xx_get_config_int1(&this->driver, config);
}

int ICM42688::getInt2(inv_icm426xx_interrupt_parameter_t *config) {
	return inv_icm426xx_get_config_int2(&this->driver, config);
}


int ICM42688::readValues() {
	return inv_icm426xx_get_data_from_registers(&this->driver);
}

int ICM42688::selfTest() {
	int rc = 0, st_result = 0;

	rc = inv_icm426xx_run_selftest(&this->driver, &st_result);

	if (rc < 0) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "An error occured while running selftest");
		return rc;
	} else {
		/* Check for GYR success (1 << 0) and ACC success (1 << 1) */
		if (st_result & 0x1)
			INV_MSG(INV_MSG_LEVEL_INFO, "Gyro Selftest PASS");
		else
			INV_MSG(INV_MSG_LEVEL_INFO, "Gyro Selftest FAIL");

		if (st_result & 0x2)
			INV_MSG(INV_MSG_LEVEL_INFO, "Accel Selftest PASS");
		else
			INV_MSG(INV_MSG_LEVEL_INFO, "Accel Selftest FAIL");
	}
	return st_result;
}