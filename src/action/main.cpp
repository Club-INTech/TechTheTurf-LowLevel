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

#include <shared/robot.hpp>

// 5 tours et une couille de loup
// + neg

// 90 deg deplié
// 165 deg rangé

void comm_thread() {
	// Grab the ref from the other core
	//ControlLoop *cl = (ControlLoop*)multicore_fifo_pop_blocking();

	// Init HL Comms on other core to handle interrupts there
	CommAction *hlComm = new CommAction(I2C_SDA, I2C_SCL, I2C_ADDR, I2C_INSTANCE);

	multicore_fifo_push_blocking((uint32_t)hlComm);

	while (true) {
		hlComm->work();
		busy_wait_us(500);
	}
}

int main() {
	// Init PicoSDK
	stdio_init_all();

	StepperDriver *elevStep = new StepperDriver(ELEVATOR_STEP, ELEVATOR_DIR, ELEVATOR_SLP, ELEVATOR_STEPS_PER_ROT, ELEVATOR_REVERSE);
	Endstop *elevEndstop = new Endstop(ELEVATOR_ENDSTOP);

	Elevator *elev = new Elevator(elevStep, elevEndstop, ELEVATOR_MM_PER_TURN, ELEVATOR_MAX_DST, ELEVATOR_MAX_PHY_DST);

	DynamixelManager *dynMan = new DynamixelManager(uart0, DYN_UART_TX, DYN_UART_RX, DYN_BAUDRATE, DYN_PROTO_VER);

	DynamixelXL430 *armDeploy = new DynamixelXL430(ARM_DEPLOY_DYN_ID);
	DynamixelXL430 *armTurn = new DynamixelXL430(ARM_TURN_DYN_ID);

	gpio_init(PUMP_PIN);
	gpio_set_dir(PUMP_PIN, true);
	gpio_put(PUMP_PIN, false);

	armDeploy->bind(dynMan);
	armTurn->bind(dynMan);

	uint16_t modelNum;

	armDeploy->ping(&modelNum);
	printf("[%i] ping %i\n", armDeploy->id, modelNum);

	armTurn->ping(&modelNum);
	printf("[%i] ping %i\n", armTurn->id, modelNum);

	multicore_launch_core1(comm_thread);

	// Send the ControlLoop ref over to the other core
	//multicore_fifo_push_blocking((uint32_t)cl);
	CommAction *hlComm = (CommAction*)multicore_fifo_pop_blocking();

	for (;;) {
		while (!hlComm->start)
			busy_wait_us(10000);

		hlComm->start = false;

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

		armDeploy->setOperatingMode(XL430OperatingMode::position);
		armTurn->setOperatingMode(XL430OperatingMode::velocity);

		armDeploy->setTorque(true);
		armDeploy->setPosition(165.0f);
		sleep_ms(1000);
		armDeploy->setPosition(90.0f);
		sleep_ms(2000);

		armTurn->setTorque(true);
		armTurn->setVelocity(60.0f);

		sleep_ms(4000);

		armDeploy->setTorque(false);
		armTurn->setTorque(false);
	}

	/*step->enable();
	for (;;) {
		step->stepCount(-1);
		if (endstop->poll())
			break;
	}

	sleep_ms(1000);

	step->rotateTurns(5);

	sleep_ms(1000);

	step->rotateTurns(-5);

	step->disable();

	mot1->setOperatingMode(XL430OperatingMode::position);
	mot2->setOperatingMode(XL430OperatingMode::velocity);

	mot1->setTorque(true);

	mot1->setPosition(165.0f);

	sleep_ms(1000);

	mot1->setPosition(90.0f);

	sleep_ms(2000);

	mot2->setTorque(true);
	mot2->setVelocity(60.0f);

	sleep_ms(4000);

	mot1->setTorque(false);
	mot2->setTorque(false);

	for (;;);*/
	
	return 0;
}