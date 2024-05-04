#include <stdio.h>
#include <pico/stdlib.h>
#include <math.h>

#include <action/dynamixel_manager.hpp>
#include <action/dynamixel_xl430.hpp>

#include <shared/robot.hpp>

static inline bool checkJumper() {
	return !gpio_get(JUMPER_PIN);
}

// Jumper in is Blue
// Jumper out is Yellow
static inline bool getSide() {
	return !gpio_get(SIDE_PIN);
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
	leftWheel->setProfileAcceleration(4292);
	rightWheel->setProfileAcceleration(4292);

	leftWheel->setTorque(true);
	rightWheel->setTorque(true);
	arm->setTorque(true);

	float leftInitial = leftWheel->getPosition();
	float rightInitial = rightWheel->getPosition();

	arm->setPosition(ARM_STANDBY_ANGLE);

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