#include <action/dynamixel_manager.hpp>

DynamixelManager::DynamixelManager(uart_inst_t *uart, uint8_t tx, uint8_t rx, uint32_t baudrate, float protoVer) {
	this->uart = uart;
	this->tx = tx;
	this->rx = rx;
	this->baudrate = baudrate;
	this->protoVer = protoVer;

	this->lastError = 0;
	this->lastHardwareError = 0;
	this->lastErrorString = nullptr;

	PortHandlerInfo info;
	info.uart_id = uart_get_index(uart);
	info.tx = tx;
	info.rx = rx;

	this->port = dynamixel::PortHandler::getPortHandler((char*)&info);
	this->handler = dynamixel::PacketHandler::getPacketHandler(protoVer);

	if (!this->port->openPort()) {
		printf("Failed to open the port!\n");
	}

	if (!this->port->setBaudRate(baudrate)) {
		printf("Failed to change the baudrate!\n");
	}
}

DynamixelManager::~DynamixelManager() {
	this->port->closePort();
	delete this->port;
}

int DynamixelManager::errorCheck() {
	if (this->lastError != COMM_SUCCESS) {
		this->lastErrorString = this->handler->getTxRxResult(this->lastError);
		return -1;
	} else if (this->lastHardwareError != 0) {
		this->lastErrorString = this->handler->getRxPacketError(this->lastHardwareError);
		return -1;
	}
	return 0;
}

int DynamixelManager::ping(uint8_t id) {
	this->lastError = this->handler->ping(this->port, id, &this->lastHardwareError);
	return errorCheck();
}

int DynamixelManager::ping(uint8_t id, uint16_t *modelNb) {
	this->lastError = this->handler->ping(this->port, id, modelNb, &this->lastHardwareError);
	return errorCheck();
}

int DynamixelManager::reboot(uint8_t id) {
	this->lastError = this->handler->reboot(this->port, id, &this->lastHardwareError);
	return errorCheck();
}

int DynamixelManager::read1(uint8_t id, uint8_t address, uint8_t *data) {
	this->lastError = this->handler->read1ByteTxRx(this->port, id, address, data, &this->lastHardwareError);
	return errorCheck();
}

int DynamixelManager::read2(uint8_t id, uint8_t address, uint16_t *data) {
	this->lastError = this->handler->read2ByteTxRx(this->port, id, address, data, &this->lastHardwareError);
	return errorCheck();
}

int DynamixelManager::read4(uint8_t id, uint8_t address, uint32_t *data) {
	this->lastError = this->handler->read4ByteTxRx(this->port, id, address, data, &this->lastHardwareError);
	return errorCheck();
}

int DynamixelManager::write1(uint8_t id, uint8_t address, uint8_t data) {
	this->lastError = this->handler->write1ByteTxRx(this->port, id, address, data, &this->lastHardwareError);
	return errorCheck();
}

int DynamixelManager::write2(uint8_t id, uint8_t address, uint16_t data) {
	this->lastError = this->handler->write2ByteTxRx(this->port, id, address, data, &this->lastHardwareError);
	return errorCheck();
}

int DynamixelManager::write4(uint8_t id, uint8_t address, uint32_t data) {
	this->lastError = this->handler->write4ByteTxRx(this->port, id, address, data, &this->lastHardwareError);
	return errorCheck();
}