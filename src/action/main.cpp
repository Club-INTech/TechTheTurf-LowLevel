#include <stdio.h>
#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <math.h>

#include <action/comm_action.hpp>

#include <action/dynamixel_manager.hpp>
#include <action/dynamixel_xl430.hpp>
#include <action/stepper_driver.hpp>
#include <action/endstop.hpp>
#include <action/elevator.hpp>
#include <action/pump.hpp>
#include <action/arm.hpp>

#include <shared/robot.hpp>

// 5 tours et une couille de loup
// + neg

// 90 deg deplié
// 165 deg rangé

void comm_thread() {
	// Grab the refs from the other core
	Elevator *elev = (Elevator*)multicore_fifo_pop_blocking();
	Arm *arm = (Arm*)multicore_fifo_pop_blocking();

	// Init HL Comms on other core to handle interrupts there
	CommAction *hlComm = new CommAction(I2C_SDA, I2C_SCL, I2C_ADDR, I2C_INSTANCE, elev, arm);

	multicore_fifo_push_blocking((uint32_t)hlComm);

	while (true) {
		hlComm->work();
		busy_wait_us(500);
	}
}

int main() {
	// Init PicoSDK
	stdio_init_all();

	StepperDriver *elevStep = new StepperDriver(ELEVATOR_STEP, ELEVATOR_DIR, ELEVATOR_EN, ELEVATOR_STEPS_PER_ROT, ELEVATOR_REVERSE);
	Endstop *elevEndstop = new Endstop(ELEVATOR_ENDSTOP);

	Elevator *elev = new Elevator(elevStep, elevEndstop, ELEVATOR_MM_PER_TURN, ELEVATOR_MAX_DST, ELEVATOR_MAX_PHY_DST);

	DynamixelManager *dynMan = new DynamixelManager(uart0, DYN_UART_TX, DYN_UART_RX, DYN_BAUDRATE, DYN_PROTO_VER);

	DynamixelXL430 *armDeploy = new DynamixelXL430(ARM_DEPLOY_DYN_ID);
	DynamixelXL430 *armTurn = new DynamixelXL430(ARM_TURN_DYN_ID);

	Pump *pump = new Pump(PUMP_PIN);
	
	armDeploy->bind(dynMan);
	armTurn->bind(dynMan);

	uint16_t modelNum;

	armDeploy->ping(&modelNum);
	printf("[%i] ping %i\n", armDeploy->id, modelNum);

	armTurn->ping(&modelNum);
	printf("[%i] ping %i\n", armTurn->id, modelNum);

	Arm *arm = new Arm(armDeploy, armTurn, ARM_DEPLOYED_ANGLE, ARM_FOLDED_ANGLE);

	multicore_launch_core1(comm_thread);

	// Send the refs over to the other core
	multicore_fifo_push_blocking((uint32_t)elev);
	multicore_fifo_push_blocking((uint32_t)arm);
	// Grab the comm ref
	CommAction *hlComm = (CommAction*)multicore_fifo_pop_blocking();

	for (;;) {
		int cmdTot;
		while ((cmdTot = hlComm->getCommand()) == -1)
			busy_wait_us(1000);

		hlComm->startWork();

		uint8_t cmd = cmdTot&0xF;
		uint8_t subcmd = (cmdTot>>4)&0xF;

		switch (cmd) {
			// Turn on/off
			case 0:
				if (hlComm->getArgumentU8(0)) {
					elev->setEnable(true);
					arm->setEnable(true);
				} else {
					elev->setEnable(false);
					arm->setEnable(false);
					pump->disable();
				}
				break;
			// Elevator control
			case 1:
				if (subcmd == 0) // Home
					elev->home();
				else if (subcmd == 1) // Absolute
					elev->moveTo(hlComm->getArgumentFloat(0));
				else if (subcmd == 2) // Relative
					elev->move(hlComm->getArgumentFloat(0));
				break;
			// Arm control
			case 2: 
				if (subcmd == 0) // Deploy
					arm->deploy();
				else if (subcmd == 1) // Fold up
					arm->fold();
				else if (subcmd == 2) // Turn head
					arm->turn(hlComm->getArgumentFloat(0));
				break;
			// Pump control
			case 3:
				if (subcmd == 0) // Pump 1
					pump->setEnable(hlComm->getArgumentU8(0) == 1);
				break;
			// Demo CMD
			case 15: 
				pump->enable();
				sleep_ms(1000);
				pump->disable();

				sleep_ms(1000);

				elev->setEnable(true);
				elev->home();
				sleep_ms(500);
				elev->moveTo(125.0f);
				sleep_ms(500);
				elev->moveTo(65.0f);
				sleep_ms(500);
				elev->moveTo(0.0f);
				sleep_ms(500);
				elev->home();
				elev->setEnable(false);

				sleep_ms(1000);

				arm->setEnable(true);
				sleep_ms(200);
				arm->fold();
				sleep_ms(1000);
				arm->deploy();
				sleep_ms(1000);
				arm->turn(360.0f);
				sleep_ms(500);
				arm->fold();

				arm->setEnable(false);
				break;
			default:
				break;
		}

		hlComm->finishWork();
	}
	
	return 0;
}