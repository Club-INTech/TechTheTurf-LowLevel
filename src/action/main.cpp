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
	Arm *rightArm = (Arm*)multicore_fifo_pop_blocking();
	Arm *leftArm = (Arm*)multicore_fifo_pop_blocking();

	// Init HL Comms on other core to handle interrupts there
	CommAction *hlComm = new CommAction(I2C_SDA, I2C_SCL, I2C_ADDR, I2C_COMM_INSTANCE, elev, rightArm, leftArm);

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

	DynamixelXL430 *rightArmDeploy = new DynamixelXL430(RIGHT_ARM_DEPLOY_DYN_ID);
	DynamixelXL430 *rightArmTurn = new DynamixelXL430(RIGHT_ARM_TURN_DYN_ID);

	DynamixelXL430 *leftArmDeploy = new DynamixelXL430(LEFT_ARM_DEPLOY_DYN_ID);
	DynamixelXL430 *leftArmTurn = new DynamixelXL430(LEFT_ARM_TURN_DYN_ID);

	Pump *pump = new Pump(PUMP0_PIN);
	Pump *pump_sol = new Pump(PUMP0_SOLENOID_PIN);
	
	rightArmDeploy->bind(dynMan);
	rightArmTurn->bind(dynMan);
	leftArmDeploy->bind(dynMan);
	leftArmTurn->bind(dynMan);

	/*uint16_t modelNum;

	rightArmDeploy->ping(&modelNum);
	printf("[%i] ping %i\n", rightArmDeploy->id, modelNum);

	rightArmTurn->ping(&modelNum);
	printf("[%i] ping %i\n", rightArmTurn->id, modelNum);*/

	Arm *rightArm = new Arm(rightArmDeploy, rightArmTurn, RIGHT_ARM_DEPLOYED_ANGLE, RIGHT_ARM_HALF_DEPLOYED_ANGLE, RIGHT_ARM_FOLDED_ANGLE);
	Arm *leftArm = new Arm(leftArmDeploy, leftArmTurn, LEFT_ARM_DEPLOYED_ANGLE, LEFT_ARM_HALF_DEPLOYED_ANGLE, LEFT_ARM_FOLDED_ANGLE);

	multicore_launch_core1(comm_thread);

	// Send the refs over to the other core
	multicore_fifo_push_blocking((uint32_t)elev);
	multicore_fifo_push_blocking((uint32_t)rightArm);
	multicore_fifo_push_blocking((uint32_t)leftArm);
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
					rightArm->setEnable(true);
					leftArm->setEnable(true);
				} else {
					elev->setEnable(false);
					rightArm->setEnable(false);
					leftArm->setEnable(false);
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
			// Right Arm control
			case 2: 
				if (subcmd == 0) // Deploy
					rightArm->deploy();
				else if (subcmd == 1) // Fold up
					rightArm->fold();
				else if (subcmd == 2) // Turn head
					rightArm->turn(hlComm->getArgumentFloat(0));
				else if (subcmd == 5)
					rightArm->halfDeploy();
				break;
			// Left Arm control
			case 3: 
				if (subcmd == 0) // Deploy
					leftArm->deploy();
				else if (subcmd == 1) // Fold up
					leftArm->fold();
				else if (subcmd == 2) // Turn head
					leftArm->turn(hlComm->getArgumentFloat(0));
				else if (subcmd == 5)
					leftArm->halfDeploy();
				break;
			// Left Pump control
			case 4:
				if (subcmd == 0) // Pump 1
					pump->setEnable(hlComm->getArgumentU8(0) == 1);
				else if (subcmd == 1) // Pump 2
					pump_sol->setEnable(hlComm->getArgumentU8(0) == 1);
				break;
			default:
				break;
		}

		hlComm->finishWork();
	}
	
	return 0;
}