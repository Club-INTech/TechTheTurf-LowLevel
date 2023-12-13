#include <stdio.h>
#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <math.h>

#include <comm.hpp>
#include <encoder.hpp>
#include <driver.hpp>
#include <control_loop.hpp>
#include <pll.hpp>
#include <accel_limiter.hpp>

// Robot definition
#define ROBOT_PAMI
#define PAMI_CARTE_2A
//#define ROBOT_DIDER

#include <robot.hpp>

void comm_thread() {
	// Grab the ref from the other core
	ControlLoop *cl = (ControlLoop*)multicore_fifo_pop_blocking();

	// Init HL Comms on other core to handle interrupts there
	Comm *hlComm = new Comm(I2C_SDA, I2C_SCL, I2C_ADDR, i2c0, cl);

	while (true) {
		hlComm->work();
		busy_wait_us(500);
	}
}

#define ASSERV_PERIOD_US 2000

int main() {
	// Init PicoSDK
	stdio_init_all();
	//sleep_ms(2000);
	//printf("Hello\n");

	// Init encoders & drivers
	Encoder *lEnc = new Encoder(LEFT_INCREMENTAL_A_PIN, LEFT_INCREMENTAL_B_PIN, ENCODER_LEFT_REVERSE, 0);
	Encoder *rEnc = new Encoder(RIGHT_INCREMENTAL_A_PIN, RIGHT_INCREMENTAL_B_PIN, ENCODER_RIGHT_REVERSE, 1);
	Driver *lDrv = new Driver(LEFT_MOTOR_FW_PIN, LEFT_MOTOR_RW_PIN, DRIVER_LEFT_REVERSE);
	Driver *rDrv = new Driver(RIGHT_MOTOR_FW_PIN, RIGHT_MOTOR_RW_PIN, DRIVER_RIGHT_REVERSE);
	//lDrv->setDutyOffset(0.15f);
	//rDrv->setDutyOffset(0.15f);

	// Init odometry
	Odometry *odo = new Odometry(ENCODER_DIST);

	// Setup PIDs
	PID *lSpeedPid = new PID(0.001f, 0.0f, 0.00005f);
	PID *rSpeedPid = new PID(0.001f, 0.0f, 0.00005f);
	PID *dstPid = new PID(10.0f, 0.5f, 0.05f);
	PID *anglePid = new PID(1000.0f, 10.0f, 5.0f);

	// Setup PLLs
	PLL *lPll = new PLL(9.0f);
	PLL *rPll = new PLL(9.0f);

	// Setup accel limiters
	AccelLimiter *dstAlim = new AccelLimiter(1000.0f);
	AccelLimiter *angleAlim = new AccelLimiter(1000000.0f);

	// Setup the controller
	Controller *ctrl = new Controller(odo);

	ControlLoop *cl = new ControlLoop(lEnc, rEnc, lDrv, rDrv, odo,
									lSpeedPid, rSpeedPid, dstPid, anglePid, lPll, rPll, dstAlim, angleAlim,
									ctrl, WHEEL_RADIUS);

	// Init motor control
	//printf("Begin\n");
	multicore_launch_core1(comm_thread);

	// Send the ControlLoop ref over to the other core
	multicore_fifo_push_blocking((uint32_t)cl);

#if 0
	while (true) {
		printf("Left Forwards\n");
		lDrv->setPwm(0.5);
		busy_wait_us(2000000);
		printf("Left Backwards\n");
		lDrv->setPwm(-0.5);
		busy_wait_us(2000000);
		lDrv->setPwm(0.0);
		printf("Right Forwards\n");
		rDrv->setPwm(0.5);
		busy_wait_us(2000000);
		printf("Right Backwards\n");
		rDrv->setPwm(-0.5);
		busy_wait_us(2000000);
		rDrv->setPwm(0.0);
		busy_wait_us(2000000);
	}
#endif

	while (true) {
		absolute_time_t start = get_absolute_time();

		cl->work();

		absolute_time_t end = get_absolute_time();

		//printf("Enc: l:%i r:%i\n", lEnc->getCount(), rEnc->getCount());

		// Try to keep the period 
		int64_t diff = absolute_time_diff_us(start, end);
		busy_wait_us(ASSERV_PERIOD_US-diff);
	}
}