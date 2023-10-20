#include <comm.hpp>
#include <hardware/gpio.h>
#include <stdio.h>

static Comm *i2cComm[2] = {nullptr, nullptr};

void dispatchSlave(i2c_inst_t *i2c, i2c_slave_event_t event) {
	uint idx = i2c_hw_index(i2c);
	if (i2cComm[idx] != nullptr)
		i2cComm[idx]->slaveHandler(i2c, event);
}

Comm::Comm(uint sdaPin, uint sclPin, uint addr, i2c_inst_t *i2c) {
	gpio_init(sdaPin);
	gpio_set_function(sdaPin, GPIO_FUNC_I2C);
	gpio_pull_up(sdaPin);

	gpio_init(sclPin);
	gpio_set_function(sclPin, GPIO_FUNC_I2C);
	gpio_pull_up(sclPin);

	i2cComm[i2c_hw_index(i2c)] = this;

	i2c_init(i2c, I2C_BAUDRATE);
	// configure I2C0 for slave mode
	i2c_slave_init(i2c, addr,  &dispatchSlave);

	resetCmd();
}

Comm::~Comm() {
	for (int i=0;i<2;i++) {
		if (i2cComm[i] == this)
			i2cComm[i] = nullptr;
	}
}

void Comm::resetCmd() {
	this->finishedRecv = false;
	this->cmdDataSize = 0;
	this->respDataSize = 0;
	for (int i=0;i<MAX_DATA_SIZE;i++) {
		this->cmdData[i] = 0;
		this->respData[i] = 0;
	}
}

void Comm::handleCmd(uint8_t *data, size_t size) {
	printf("handle size: %i\n", size);
	this->respDataSize = size;
	for (size_t i=0;i<size;i++) {
		//printf("%i: %i\n", i, data[i]);
		this->respData[i] = data[i];
	}
}

void Comm::slaveHandler(i2c_inst_t *i2c, i2c_slave_event_t event) {
	size_t nb, i;
	uint8_t byte;
	switch (event) {
		// Data from master is available for reading. Slave must read from Rx FIFO.
		case I2C_SLAVE_RECEIVE:
			nb = i2c_get_read_available(i2c);
			//printf("Rcv av:%i\n", nb);
			if (nb < 1) // ?????
				return;
			for (size_t i=0;i<nb;i++) {
				byte = i2c_read_byte_raw(i2c);
				if (this->cmdDataSize >= MAX_DATA_SIZE) // ?????
					return;
				this->cmdData[this->cmdDataSize++] = byte;
			}
			break;
		// Master is requesting data. Slave must write into Tx FIFO.
		case I2C_SLAVE_REQUEST:
			printf("Slave req\n");
			this->finishedRecv = true;
			if (!this->finishedRecv) { // ?????
				resetCmd();
				return;
			}
			handleCmd((uint8_t*)&this->cmdData, this->cmdDataSize);
			//printf("after\n");
			for (i=0;i<this->respDataSize;i++) {
				do {
					nb = i2c_get_write_available(i2c);
				} while (nb == 0);
				i2c_write_byte_raw(i2c, this->respData[i]);
			}
			//printf("wrote\n");
			resetCmd();
			break;
		// Master has sent a Stop or Restart signal. Slave may prepare for the next transfer.
		case I2C_SLAVE_FINISH:
			printf("Slave finish TX\n");
			this->finishedRecv = true;
			break;
		default:
			break; 
	}
}