#include <asserv/comm_bg.hpp>

#define SERIAL_HEADER 0x4142
#define COMM_BAUD 1000000

CommBG::CommBG(uint8_t uid, uart_inst_t *uart, uint8_t tx, uint8_t rx) {
	this->uid = uid;
	this->uart = uart;
	this->tx = tx;
	this->rx = rx;
	uart_init(uart, COMM_BAUD);

	// Set the TX and RX pins by using the function select on the GPIO
	// Set datasheet for more information on function select
	gpio_set_function(tx, GPIO_FUNC_UART);
	gpio_set_function(rx, GPIO_FUNC_UART);
}

CommBG::~CommBG() {
	uart_deinit(this->uart);

	gpio_set_function(this->tx, GPIO_FUNC_NULL);
	gpio_set_function(this->rx, GPIO_FUNC_NULL);
}

void CommBG::enable() {
	this->sendCmd(0x1);
}

void CommBG::disable() {
	this->sendCmd(0x0);
}

void CommBG::setTarget(float target) {
	this->sendCmd(0x2);
	uart_write_blocking(this->uart, (uint8_t*)&target, sizeof(float));
}

void CommBG::setMotionControl(uint8_t type) {
	this->sendCmd(0x3);
	uart_putc_raw(this->uart, type);
}

void CommBG::sendCmd(uint8_t cmd) {
	// Little endian -> flip around
	uint16_t val = ((SERIAL_HEADER&0xFF) << 8) | ((SERIAL_HEADER>>8) & 0xFF);
	uart_write_blocking(this->uart, (uint8_t*)&val, sizeof(uint16_t));
	uart_putc_raw(this->uart, this->uid);
	uart_putc_raw(this->uart, cmd);
}
