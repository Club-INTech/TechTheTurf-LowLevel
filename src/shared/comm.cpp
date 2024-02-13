#include <hardware/gpio.h>
#include <stdio.h>
#include <cstring>

#include <shared/comm.hpp>

#include <shared/cppcrc.h>

static Comm *i2cComm[2] = {nullptr, nullptr};

void dispatchSlave(i2c_inst_t *i2c, i2c_slave_event_t event) {
	uint idx = i2c_hw_index(i2c);
	if (i2cComm[idx] != nullptr)
		i2cComm[idx]->slaveHandler(event);
}

void Comm::i2cInit(uint8_t address) {
	uint idx = i2c_hw_index(i2c);
	i2cComm[idx] = this;

	gpio_init(this->sdaPin);
	gpio_set_function(this->sdaPin, GPIO_FUNC_I2C);
	gpio_pull_up(this->sdaPin);

	gpio_init(this->sclPin);
	gpio_set_function(this->sclPin, GPIO_FUNC_I2C);
	gpio_pull_up(this->sclPin);

	i2c_init(this->i2c, I2C_BAUDRATE);

	// configure i2c for slave mode
	i2c_slave_init(i2c, address,  &dispatchSlave);
}

void Comm::i2cDeinit() {
	i2c_slave_deinit(this->i2c);

	i2c_deinit(this->i2c);

	gpio_deinit(this->sdaPin);
	gpio_deinit(this->sclPin);

	uint idx = i2c_hw_index(this->i2c);
	i2cComm[idx] = nullptr;
}

Comm::Comm(uint sdaPin, uint sclPin, uint addr, i2c_inst_t *i2c) {
	this->sdaPin = sdaPin;
	this->sclPin = sclPin;
	this->i2c = i2c;

	i2cInit(addr);

	resetCmd();
	clearTelems();
}

Comm::~Comm() {
	i2cDeinit();
}

void Comm::resetRecvCmd() {
	this->recvDataSize = 0;
	memset(this->recvData, 0, MAX_DATA_SIZE);
}

void Comm::resetSendCmd() {
	this->sendDataSize = 0;
	memset(this->sendData, 0, MAX_DATA_SIZE);
}

void Comm::resetCmd() {
	resetRecvCmd();
	resetSendCmd();
}

void Comm::clearTelems() {
	for (size_t i=0; i<MAX_TELEMETRY_NB; i++)
		this->telems[i] = nullptr;
}

TelemetryBase* Comm::getTelem(uint8_t idx) {
	if (idx >= MAX_TELEMETRY_NB)
		return nullptr;
	return this->telems[idx];
}

void Comm::addTelem(uint8_t idx, TelemetryBase *telem) {
	if (idx >= MAX_TELEMETRY_NB)
		return;
	this->telems[idx] = telem;
}

void Comm::work() {
	for (size_t idx=0; idx<MAX_TELEMETRY_NB; idx++) {
		TelemetryBase *telem = this->telems[idx];
		if (!telem)
			continue;
		size_t size = telem->elemSize();
		
		while (telem->size() > 0) {
			telem->getInBuffer(&this->serialBuffer[0], MAX_DATA_SIZE, idx);
			putchar('\xDE');
			putchar('\xAD');
			for (size_t i=0; i<size; i++)
				putchar(this->serialBuffer[i]);
			uint32_t crc = CRC32::CRC32::calc(&this->serialBuffer[0], size);
			for (size_t i=0; i<4; i++)
				putchar(((uint8_t*)(&crc))[i]);
		}
	}
}

void Comm::slaveHandler(i2c_slave_event_t event) {
	size_t nb;
	uint8_t byte;
	switch (event) {
		// Data from master is available for reading. Slave must read from Rx FIFO.
		case I2C_SLAVE_RECEIVE:
			nb = i2c_get_read_available(this->i2c);
			//printf("Rcv av:%i\n", nb);
			if (nb < 1) // ?????
				return;

			for (size_t i=0;i<nb;i++) {
				byte = i2c_read_byte_raw(this->i2c);
				if (this->recvDataSize >= MAX_DATA_SIZE) // ?????
					return;
				this->recvData[this->recvDataSize++] = byte;
			}

			break;
		// Master is requesting data. Slave must write into Tx FIFO.
		case I2C_SLAVE_REQUEST:
			//printf("Slave req\n");
			if (this->sendDataSize == 0) // ?????
				return;

			i2c_write_raw_blocking(this->i2c, (uint8_t*)&this->sendData, this->sendDataSize);

			resetSendCmd();
			break;
		// Master has sent a Stop or Restart signal. Slave may prepare for the next transfer.
		case I2C_SLAVE_FINISH:
			//printf("Slave finish TX\n");

			if (this->recvDataSize == 0) // ?????
				return;

			resetSendCmd();

			handleCmd((uint8_t*)&this->recvData, this->recvDataSize);

			resetRecvCmd();
			break;
		default:
			break; 
	}
}