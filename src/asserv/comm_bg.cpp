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

	mutex_init(&this->mutex);
}

CommBG::~CommBG() {
	uart_deinit(this->uart);

	gpio_set_function(this->tx, GPIO_FUNC_NULL);
	gpio_set_function(this->rx, GPIO_FUNC_NULL);
}

// Not protected with mutex
void CommBG::sendCmd(uint8_t cmd) {
	// Little endian -> flip around
	uint16_t val = ((SERIAL_HEADER&0xFF) << 8) | ((SERIAL_HEADER>>8) & 0xFF);
	uart_write_blocking(this->uart, (uint8_t*)&val, sizeof(uint16_t));
	uart_putc_raw(this->uart, this->uid);
	uart_putc_raw(this->uart, cmd);
}

void CommBG::enable() {
	mutex_try_enter(&this->mutex, nullptr);
	this->sendCmd(0x1);
	mutex_exit(&this->mutex);
}

void CommBG::disable() {
	mutex_try_enter(&this->mutex, nullptr);
	this->sendCmd(0x0);
	mutex_exit(&this->mutex);
}

void CommBG::setTarget(float target) {
	mutex_try_enter(&this->mutex, nullptr);
	this->sendCmd(0x2);
	uart_write_blocking(this->uart, (uint8_t*)&target, sizeof(float));
	mutex_exit(&this->mutex);
}

void CommBG::setMotionControl(uint8_t type) {
	mutex_try_enter(&this->mutex, nullptr);
	this->sendCmd(0x3);
	uart_putc_raw(this->uart, type);
	mutex_exit(&this->mutex);
}

int CommBG::readStats(float *vel, float *current, float *temp, float *vbus) {
	if (vel) *vel = 0;
	if (current) *current = 0;
	if (temp) *temp = 0;
	if (vbus) *vbus = 0;

	mutex_try_enter(&this->mutex, nullptr);
	this->sendCmd(0x4);
	uint16_t last = 0;

	int cnt = 0;

	while (uart_is_readable_within_us(this->uart, RESPONSE_TIMEOUT)) {
		uint8_t by = uart_getc(this->uart);
		last = by | (last << 8);
		if (last != SERIAL_HEADER)
			continue;

		by = uart_getc(this->uart);
		if (by != this->uid)
			continue;

		by = uart_getc(this->uart);

		if (by != 0x84)
			break;

		float vals[4];
		uart_read_blocking(this->uart, (uint8_t*)&vals, sizeof(float)*4);


		if (vel) {*vel = vals[0];cnt++;}
		if (current) {*current = vals[1];cnt++;}
		if (temp) {*temp = vals[2];cnt++;}
		if (vbus) {*vbus = vals[3];cnt++;}
	}

	this->sendCmd(0x7F);
	mutex_exit(&this->mutex);

	return cnt;
}