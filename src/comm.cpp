#include <comm.hpp>
#include <hardware/gpio.h>
#include <stdio.h>
#include <cstring>

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

	// configure I2C0 for slave mode
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

void Comm::handleCmd(uint8_t *data, size_t size) {
	printf("handle size: %i\n", size);
	if (size < 1)
		return;
	if (data[0] == 0)
		return;
	this->sendDataSize = size;
	for (size_t i=0;i<size;i++) {
		//printf("%i: %i\n", i, data[i]);
		this->sendData[i] = data[i];
	}
}

void Comm::slaveHandler(i2c_slave_event_t event) {
	size_t nb, i;
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
			printf("Slave req\n");
			if (this->sendDataSize == 0) // ?????
				return;

			for (i=0;i<this->sendDataSize;i++) {
				do {
					nb = i2c_get_write_available(this->i2c);
				} while (nb == 0);

				i2c_write_byte_raw(this->i2c, this->sendData[i]);
			}

			resetSendCmd();
			break;
		// Master has sent a Stop or Restart signal. Slave may prepare for the next transfer.
		case I2C_SLAVE_FINISH:
			printf("Slave finish TX\n");

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