#include <stdio.h>
#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <math.h>

#include <action/comm_action.hpp>

#include <action/dynamixel_manager.hpp>
#include <action/dynamixel_xl430.hpp>

#include <shared/robot.hpp>

#define BAUDRATE 57600
#define PROTO_VER 2.0
#define DXL_ID 5
#define DXL_ID2 11

int main() {
	// Init PicoSDK
	stdio_init_all();
	sleep_ms(2000);

	DynamixelManager *man = new DynamixelManager(uart0, 12, 13, BAUDRATE, PROTO_VER);

	DynamixelXL430 *mot1 = new DynamixelXL430(DXL_ID);
	DynamixelXL430 *mot2 = new DynamixelXL430(DXL_ID2);

	mot1->bind(man);
	mot2->bind(man);

	uint16_t modelNum;

	mot1->ping(&modelNum);
	printf("[%i] ping %i\n", mot1->id, modelNum);

	mot2->ping(&modelNum);
	printf("[%i] ping %i\n", mot2->id, modelNum);

	mot1->setOperatingMode(XL430OperatingMode::position);
	mot2->setOperatingMode(XL430OperatingMode::velocity);

	mot1->setTorque(true);
	mot2->setTorque(true);

	for (int i=0; i<2; i++) {
		printf("Turn %i\n", i);
		mot1->setPositionTarget(0.0);
		sleep_ms(2000);
		mot1->setPositionTarget(1.0);
		sleep_ms(2000);
		mot2->setVelocityTarget(-1.0);
		sleep_ms(2000);
		mot2->setVelocityTarget(1.0);
		sleep_ms(2000);
		mot2->setVelocityTarget(0.0);
	}

	mot1->setTorque(false);
	mot2->setTorque(false);

	delete mot1;
	delete mot2;
	delete man;
	
	return 0;
}