#include <asserv/comm_odrive.hpp>
#include <cstdarg>
#include <pico/printf.h>

#define COMM_BAUD 1000000

CommODrive::CommODrive(uart_inst_t *uart, uint8_t tx, uint8_t rx) {
	this->uart = uart;
	this->tx = tx;
	this->rx = rx;
	uart_init(uart, COMM_BAUD);
	uart_set_translate_crlf(uart, false);

	// Set the TX and RX pins by using the function select on the GPIO
	// Set datasheet for more information on function select
	gpio_set_function(tx, GPIO_FUNC_UART);
	gpio_set_function(rx, GPIO_FUNC_UART);
}

CommODrive::~CommODrive() {
	uart_deinit(this->uart);

	gpio_set_function(this->tx, GPIO_FUNC_NULL);
	gpio_set_function(this->rx, GPIO_FUNC_NULL);
}

void CommODrive::clearErrors() {
	uart_puts(this->uart, "sc\n");
}

void CommODrive::reboot() {
	uart_puts(this->uart, "sr\n");
}

// TODO
bool CommODrive::hasErrors() {
	return false;
}

float CommODrive::getVbus() {
	return 0;
}

// In turns/s
float CommODrive::getVelocity() {
	return 0;
}
// In turns
float CommODrive::getPosition() {
	return 0;
}

void CommODrive::setAxisState(uint8_t axis, ODrive::AxisState state) {
	uartPrintf("w axis%i.requested_state %i\n", axis, state);
}

void CommODrive::setAxisControlMode(uint8_t axis, ODrive::ControlMode mode) {
	uartPrintf("w axis%i.controller.config.control_mode %i\n", axis, mode);
}

// Velocity is in turns/s
void CommODrive::setTargetVelocity(uint8_t axis, float vel) {
	uartPrintf("v %i %f\n", axis, vel);
}

void CommODrive::requestFeedback(float *pos, float *vel) {
	//sscanf()
}

static char tmpBuffer[160];

void CommODrive::uartPrintf(const char* fmt, ...) {
	va_list arg;
    va_start(arg, fmt);
    vsnprintf((char*)&tmpBuffer, 160, fmt, arg);
    va_end(arg);
    uart_puts(this->uart, tmpBuffer);
}