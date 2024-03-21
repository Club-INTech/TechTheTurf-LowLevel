#include <stdio.h>
#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <math.h>

#include <asserv/comm_asserv.hpp>
#include <asserv/encoder.hpp>
#include <asserv/driver.hpp>
#include <asserv/control_loop.hpp>
#include <asserv/pll.hpp>
#include <asserv/accel_limiter.hpp>
#include <asserv/comm_bg.hpp>
#include <asserv/comm_odrive.hpp>
#include <asserv/driver_bg.hpp>
#include <asserv/driver_odrive.hpp>

#include <shared/robot.hpp>

void comm_thread() {
	// Grab the ref from the other core
	ControlLoop *cl = (ControlLoop*)multicore_fifo_pop_blocking();

	// Init HL Comms on other core to handle interrupts there
	CommAsserv *hlComm = new CommAsserv(I2C_SDA, I2C_SCL, I2C_ADDR, I2C_INSTANCE, cl);

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

#ifdef ROBOT_MAIN
#ifdef MAIN_USE_ODRIVE
	CommODrive *odrive = new CommODrive(UART_INSTANCE, UART_TX, UART_RX);

	DriverODrive *lDrv = new DriverODrive(odrive, ODRIVE_LEFT_AXIS, DRIVER_LEFT_REVERSE);
	DriverODrive *rDrv = new DriverODrive(odrive, ODRIVE_RIGHT_AXIS, DRIVER_RIGHT_REVERSE);
#else
	CommBG *lBg = new CommBG(BG_LEFT_ID, UART_INSTANCE, UART_TX, UART_RX);
	CommBG *rBg = new CommBG(BG_RIGHT_ID, UART_INSTANCE, UART_TX, UART_RX);

	DriverBG *lDrv = new DriverBG(lBg, DRIVER_LEFT_REVERSE);
	DriverBG *rDrv = new DriverBG(rBg, DRIVER_RIGHT_REVERSE);
#endif
	// Temp hack to bypass speed PID
	float speedMul = 0.0f;
#else
	Driver *lDrv = new Driver(LEFT_MOTOR_FW_PIN, LEFT_MOTOR_RW_PIN, DRIVER_LEFT_REVERSE);
	Driver *rDrv = new Driver(RIGHT_MOTOR_FW_PIN, RIGHT_MOTOR_RW_PIN, DRIVER_RIGHT_REVERSE);
	float speedMul = 1.0f;
#endif

	//lDrv->setDutyOffset(0.15f);
	//rDrv->setDutyOffset(0.15f);

	// Init odometry
	Odometry *odo = new Odometry(ENCODER_DIST);

	// Setup PIDs
	PID *lSpeedPid = new PID(SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD);
	PID *rSpeedPid = new PID(SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD);

	PID *dstPid = new PID(DST_PID_KP, DST_PID_KI, DST_PID_KD);
	PID *anglePid = new PID(ANGLE_PID_KP, ANGLE_PID_KI, ANGLE_PID_KD);

#ifndef ROBOT_MAIN
	// All other robots use PWM
	lSpeedPid->setClamp(-1.0f, 1.0f);
	rSpeedPid->setClamp(-1.0f, 1.0f);
#else
	// The main robot uses the BG, so the speed pid has 1 gain and clamps to max vel
	lSpeedPid->setClamp(-MAX_VELOCITY, MAX_VELOCITY);
	rSpeedPid->setClamp(-MAX_VELOCITY, MAX_VELOCITY);
#endif

	//float maxVal = 1.0f/lSpeedPid->Kp;
	//dstPid->setClamp(-maxVal,maxVal);
	//anglePid->setClamp(-maxVal,maxVal);

	dstPid->setClamp(-3000.0f, 3000.0f);
	anglePid->setClamp(-100000.0f, 100000.0f);

	// Setup PLLs
	PLL *lPll = new PLL(9.0f);
	PLL *rPll = new PLL(9.0f);

	// Setup accel limiters
	AccelLimiter *lSpeedAlim = new AccelLimiter(MAX_ACCEL);
	AccelLimiter *rSpeedAlim = new AccelLimiter(MAX_ACCEL);

	// Setup trapezoidal speed profile
	SpeedProfile *speedProfile = new SpeedProfile(MAX_LIN_VELOCITY, MAX_LIN_ACCEL);

	// Setup the controller
	Controller *ctrl = new Controller(odo, speedProfile);

	ControlLoop *cl = new ControlLoop(lEnc, rEnc, lDrv, rDrv, odo,
									lSpeedPid, rSpeedPid, dstPid, anglePid, lPll, rPll, lSpeedAlim, rSpeedAlim,
									ctrl, ENCODER_WHEEL_RADIUS, POSITION_DOWNSAMPLING, speedMul);

	// Init motor control
	//printf("Begin\n");
	multicore_launch_core1(comm_thread);

	// Send the ControlLoop ref over to the other core
	multicore_fifo_push_blocking((uint32_t)cl);

	while (true) {
		absolute_time_t start = get_absolute_time();

		cl->work();

		absolute_time_t end = get_absolute_time();

		// Try to keep the period 
		int64_t diff = absolute_time_diff_us(start, end);
		busy_wait_us(ASSERV_PERIOD_US-diff);
	}
}