#pragma once

#include <cstdint>
#include <shared/i2c_device.hpp>

enum class INA236Scale {
	s81_92mV = 0b0, // Default
	s20_48mV = 0b1
};

enum class INA236Avg {
	a1 = 0b000, // Default
	a4 = 0b001,
	a16 = 0b010,
	a64 = 0b011,
	a128 = 0b100,
	a256 = 0b101,
	a512 = 0b110,
	a1024 = 0b111,
};

enum class INA236CT {
	c140us = 0b000,
	c204us = 0b001,
	c332us = 0b010,
	c588us = 0b011,
	c1100us = 0b100, // Default
	c2116us = 0b101,
	c4156us = 0b110,
	c8244us = 0b111,
};

enum class INA236Mode {
	Shutdown = 0b000,
	TriggeredShunt = 0b001,
	TriggeredBus = 0b010,
	TriggeredShuntBus = 0b011,
	Shutdown2 = 0b100,
	ContinuousShunt = 0b101,
	ContinuousBus = 0b110,
	ContinuousShuntBus = 0b111, // Default
};

// Alert not implemented as not used here
class INA236 : public I2CDevice
{
public:
	INA236(i2c_inst_t *inst, uint8_t sda, uint8_t scl, uint8_t addr, float shuntResistance, float maxCurrent, uint baudrate=100e3, bool pullUp=true, uint timeoutUs=1000);

	// Will erase all config, will only set back the calibration value
	void reset();

	// Ohms
	void setShuntResistance(float res);
	// Amps
	void setMaxCurrent(float curr);

	void setAdcScale(INA236Scale scale, bool apply=true);
	void setAveraging(INA236Avg avg, bool apply=true);
	// CT = Conversion Time 
	void setBusCT(INA236CT ct, bool apply=true);
	void setShuntCT(INA236CT ct, bool apply=true);
	void setMode(INA236Mode mode, bool apply=true);

	// All in one set config
	void setConfig(INA236Scale scale = INA236Scale::s81_92mV, INA236Avg avg = INA236Avg::a1, INA236CT busCt = INA236CT::c1100us,
					INA236CT shuntCt = INA236CT::c1100us, INA236Mode mode = INA236Mode::ContinuousShuntBus);

	float readShuntVoltage(); // In Volts
	float readBusVoltage(); // In Volts
	float readPower(); // In Watts
	float readCurrent(); // In Amps 

	// Should return 0x5449 or 'TI'
	uint16_t readManufacturerID();
	// Should return 0xA080
	uint16_t readDeviceID();

private:
	// Will also write back value to chip
	void updateCalibration();
	// Will write current config to chip
	void writeConfig();
	// Will read current config from chip
	void readConfig();

	// Derived from maxCurrent & shuntRes
	float currentLsb;

	uint16_t config;

	float maxCurrent;
	float shuntRes;
};