/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Ryu Woon Jung (Leon) */

#include "dynamixel_sdk/port_handler_pico.h"

#include <pico/stdlib.h>
#include <stdio.h>
#include <cstring>

#define LATENCY_TIMER     4  // msec (USB latency timer)

using namespace dynamixel;

PortHandlerPico::PortHandlerPico(const char *port_name)
	: baudrate_(DEFAULT_BAUDRATE_),
		packet_start_time_(0.0),
		packet_timeout_(0.0),
		tx_time_per_byte(0.0)
{
	is_using_ = false;
	setPortName(port_name);

	if (port_name[0] == 0) {
		this->uart = uart0;
	} else if (port_name[0] == 1) {
		this->uart = uart1;
	}

	this->tx = port_name[1];
	this->rx = port_name[2];

	setTxDisable();
}

bool PortHandlerPico::openPort()
{
	return setBaudRate(baudrate_);
}

void PortHandlerPico::closePort()
{
	uart_deinit(this->uart);

	gpio_set_function(this->tx, GPIO_FUNC_NULL);
	gpio_set_function(this->rx, GPIO_FUNC_NULL);
}

void PortHandlerPico::clearPort()
{
	int temp __attribute__((unused));
	while (uart_is_readable(this->uart)) {
		temp = uart_getc(this->uart);
	}
}

void PortHandlerPico::setPortName(const char *port_name)
{
	strcpy(port_name_, port_name);
}

char *PortHandlerPico::getPortName()
{
	return port_name_;
}

bool PortHandlerPico::setBaudRate(const int baudrate)
{
	baudrate_ = checkBaudrateAvailable(baudrate);

	if (baudrate_ == -1)
		return false;

	setupPort(baudrate_);

	return true;
}

int PortHandlerPico::getBaudRate()
{
	return baudrate_;
}

int PortHandlerPico::getBytesAvailable()
{
	return uart_is_readable(this->uart) ? 1 : 0;
}

int PortHandlerPico::readPort(uint8_t *packet, int length)
{
	int rx_length = 0;

	while (rx_length < length && uart_is_readable(this->uart))
	{
		packet[rx_length++] = uart_getc(this->uart);
	}

	return rx_length;
}

int PortHandlerPico::writePort(uint8_t *packet, int length)
{
	setTxEnable();

	uart_write_blocking(this->uart, packet, length);
	uart_tx_wait_blocking(this->uart);	
	/*for (int i=0; i<length; i++)
		uart_getc(this->uart);*/

	setTxDisable();

	return length;
}

void PortHandlerPico::setPacketTimeout(uint16_t packet_length)
{
	packet_start_time_  = getCurrentTime();
	packet_timeout_     = (tx_time_per_byte * (double)packet_length) + (LATENCY_TIMER * 2.0) + 2.0;
}

void PortHandlerPico::setPacketTimeout(double msec)
{
	packet_start_time_  = getCurrentTime();
	packet_timeout_     = msec;
}

bool PortHandlerPico::isPacketTimeout()
{
	if (getTimeSinceStart() > packet_timeout_)
	{
		packet_timeout_ = 0;
		return true;
	}

	return false;
}

double PortHandlerPico::getCurrentTime()
{
	absolute_time_t abstime = get_absolute_time();
	uint32_t time = to_ms_since_boot(abstime);
	return ((double)time);
}

double PortHandlerPico::getTimeSinceStart()
{
	double elapsed_time;

	elapsed_time = getCurrentTime() - packet_start_time_;
	if (elapsed_time < 0.0)
		packet_start_time_ = getCurrentTime();

	return elapsed_time;
}

bool PortHandlerPico::setupPort(int baudrate)
{
	uart_init(this->uart, baudrate);

	// Set the TX and RX pins by using the function select on the GPIO
	// Set datasheet for more information on function select
	gpio_set_function(this->tx, GPIO_FUNC_UART);
	gpio_set_function(this->rx, GPIO_FUNC_UART);

	sleep_ms(100);

	tx_time_per_byte = (1000.0 / (double)baudrate) * 10.0;
	return true;
}

int PortHandlerPico::checkBaudrateAvailable(int baudrate)
{
	switch(baudrate)
	{
		case 9600:
			return 9600;
		case 57600:
			return 57600;
		case 115200:
			return 115200;
		case 1000000:
			return 1000000;
		case 2000000:
			return 2000000;
		case 3000000:
			return 3000000;
		case 4000000:
			return 4000000;
		case 4500000:
			return 4500000;
		default:
			return -1;
	}
}

void PortHandlerPico::setPowerOn()
{
}

void PortHandlerPico::setPowerOff()
{
}

void PortHandlerPico::setTxEnable()
{
	gpio_set_function(this->rx, GPIO_FUNC_NULL);
	gpio_set_function(this->tx, GPIO_FUNC_UART);
}

void PortHandlerPico::setTxDisable()
{
	gpio_set_function(this->rx, GPIO_FUNC_UART);
	gpio_set_function(this->tx, GPIO_FUNC_NULL);
	//uart_tx_wait_blocking(this->uart);
}
