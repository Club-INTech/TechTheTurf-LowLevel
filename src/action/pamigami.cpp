#include <stdio.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <math.h>

#include <action/dynamixel_manager.hpp>
#include <action/dynamixel_xl430.hpp>
#include <action/hcsr04.hpp>

#include <shared/robot.hpp>

static inline bool checkJumper() {
	return !gpio_get(JUMPER_PIN);
}

// Jumper in is Blue
// Jumper out is Yellow
static inline bool getSide() {
	return !gpio_get(SIDE_PIN);
}

void range_thread() {
	// Grab the refs from the other core
	HCSR04 *hc = new HCSR04(HCSR04_TRIG, HCSR04_ECHO);
	DynamixelXL430 *leftWheel = (DynamixelXL430*)multicore_fifo_pop_blocking();
	DynamixelXL430 *rightWheel = (DynamixelXL430*)multicore_fifo_pop_blocking();
	DynamixelXL430 *arm = (DynamixelXL430*)multicore_fifo_pop_blocking();

	// Wait for start
	multicore_fifo_pop_blocking();

	while (true) {
		float dst = hc->getDistance();
		printf("%f\n", dst);
		if (dst <= ESTOP_DIST) {
			for (;;) {
				leftWheel->setTorque(false);
				rightWheel->setTorque(false);
				arm->setTorque(false);
				busy_wait_us(10000);
			}
		}

	}
}
int main() {
	// Init PicoSDK
	stdio_init_all();

	// Init jumper & side pin
	gpio_init(JUMPER_PIN);
	gpio_init(SIDE_PIN);
	gpio_set_dir(JUMPER_PIN, GPIO_IN);
	gpio_set_dir(SIDE_PIN, GPIO_IN);
	gpio_pull_up(JUMPER_PIN);
	gpio_pull_up(SIDE_PIN);

	// Init Dynamixels
	DynamixelManager *dynMan = new DynamixelManager(uart0, DYN_UART_TX, DYN_UART_RX, DYN_BAUDRATE, DYN_PROTO_VER);

	DynamixelXL430 *leftWheel = new DynamixelXL430(LEFT_WHEEL_DYN_ID);
	DynamixelXL430 *rightWheel = new DynamixelXL430(RIGHT_WHEEL_DYN_ID);
	DynamixelXL430 *arm = new DynamixelXL430(ARM_DYN_ID);

	leftWheel->bind(dynMan);
	rightWheel->bind(dynMan);
	arm->bind(dynMan);

	leftWheel->setOperatingMode(XL430OperatingMode::extendedPosition);
	rightWheel->setOperatingMode(XL430OperatingMode::extendedPosition);
	arm->setOperatingMode(XL430OperatingMode::position);

	leftWheel->setProfileVelocity(55.0f);
	rightWheel->setProfileVelocity(56.0f);
	leftWheel->setProfileAcceleration(4292.0f);
	rightWheel->setProfileAcceleration(4292.0f);

	leftWheel->setTorque(true);
	rightWheel->setTorque(true);
	arm->setTorque(true);

	float leftInitial = leftWheel->getPosition();
	float rightInitial = rightWheel->getPosition();

	arm->setPosition(ARM_STANDBY_ANGLE);

	multicore_launch_core1(range_thread);

	// Send the refs over to the other core
	multicore_fifo_push_blocking((uint32_t)leftWheel);
	multicore_fifo_push_blocking((uint32_t)rightWheel);
	multicore_fifo_push_blocking((uint32_t)arm);

	/*uint16_t modelNum;

	leftWheel->ping(&modelNum);
	printf("[%i] ping %i\n", leftWheel->id, modelNum);

	rightWheel->ping(&modelNum);
	printf("[%i] ping %i\n", rightWheel->id, modelNum);

	arm->ping(&modelNum);
	printf("[%i] ping %i\n", arm->id, modelNum);*/

	// Only on falling edge
	for (;!checkJumper(););
	busy_wait_us(100000);
	for (;checkJumper(););

	busy_wait_us(91000000);

	// Start
	//multicore_fifo_push_blocking(1);

	leftWheel->setPosition(leftInitial+ANGLES_TO_RUN);
	rightWheel->setPosition(rightInitial-ANGLES_TO_RUN);
	busy_wait_us(400000);
	while (leftWheel->isMoving() || rightWheel->isMoving())
		busy_wait_us(5000);
	
	arm->setPosition(getSide() ? ARM_BLUE_ANGLE : ARM_YELLOW_ANGLE);
	busy_wait_us(400000);
	while (arm->isMoving())
		busy_wait_us(5000);

	for (;;) {
		leftWheel->setLED(true);
		busy_wait_us(50000);
		leftWheel->setLED(false);
		rightWheel->setLED(true);
		busy_wait_us(50000);
		rightWheel->setLED(false);
		arm->setLED(true);
		busy_wait_us(50000);
		arm->setLED(false);
	}
	
	return 0;

}