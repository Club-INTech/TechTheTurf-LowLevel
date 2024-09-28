#include "asserv/effects.hpp"
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

#ifdef ENABLE_EFFECTS
	Effects *effects = (Effects*)multicore_fifo_pop_blocking();
#else
	Effects *effects = nullptr;
#endif

	// Init HL Comms on other core to handle interrupts there
	CommAsserv *hlComm = new CommAsserv(I2C_SDA, I2C_SCL, I2C_ADDR, I2C_INSTANCE, cl, effects);

	while (true) {
		hlComm->work();
		busy_wait_us(500);
	}
}

int main() {
	// Set overclock
	//set_sys_clock_khz(240000, true);

	// Init PicoSDK
	stdio_init_all();

	// Init Encoders
	Encoder *lEnc = new Encoder(LEFT_INCREMENTAL_A_PIN, LEFT_INCREMENTAL_B_PIN, ENCODER_LEFT_REVERSE, 0);
	Encoder *rEnc = new Encoder(RIGHT_INCREMENTAL_A_PIN, RIGHT_INCREMENTAL_B_PIN, ENCODER_RIGHT_REVERSE, 1);

	// Init Motor Drivers
#ifdef ROBOT_MAIN

#ifdef ROBOT_MAIN_ODRIVE
	CommODrive *odrive = new CommODrive(UART_INSTANCE, UART_TX, UART_RX);

	DriverODrive *lDrv = new DriverODrive(odrive, ODRIVE_LEFT_AXIS, DRIVER_LEFT_REVERSE);
	DriverODrive *rDrv = new DriverODrive(odrive, ODRIVE_RIGHT_AXIS, DRIVER_RIGHT_REVERSE);
#else
	CommBG *lBg = new CommBG(BG_LEFT_ID, UART_INSTANCE, UART_TX, UART_RX);
	CommBG *rBg = new CommBG(BG_RIGHT_ID, UART_INSTANCE, UART_TX, UART_RX);

	DriverBG *lDrv = new DriverBG(lBg, DRIVER_LEFT_REVERSE);
	DriverBG *rDrv = new DriverBG(rBg, DRIVER_RIGHT_REVERSE);
#endif

#else // ! ROBOT_MAIN
	Driver *lDrv = new Driver(LEFT_MOTOR_FW_PIN, LEFT_MOTOR_RW_PIN, DRIVER_LEFT_REVERSE);
	Driver *rDrv = new Driver(RIGHT_MOTOR_FW_PIN, RIGHT_MOTOR_RW_PIN, DRIVER_RIGHT_REVERSE);
	lDrv->setDutyOffset(DRIVER_DUTY_OFFSET);
	rDrv->setDutyOffset(DRIVER_DUTY_OFFSET);
#endif

	// Init odometry
	Odometry *odo = new Odometry(ENCODER_DIST);

	// Setup PIDs
	PID *lSpeedPid = new PID(SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD);
	PID *rSpeedPid = new PID(SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD);

	PID *dstPid = new PID(DST_PID_KP, DST_PID_KI, DST_PID_KD);
	PID *anglePid = new PID(ANGLE_PID_KP, ANGLE_PID_KI, ANGLE_PID_KD);

	dstPid->setClamp(-DST_PID_CLAMP, DST_PID_CLAMP);
	anglePid->setClamp(-ANGLE_PID_CLAMP, ANGLE_PID_CLAMP);

#ifdef ROBOT_MAIN
	// The main robot uses the BG/ODrive, so the speed pid has 1 gain and clamps to max vel
	lSpeedPid->setClamp(-MAX_VELOCITY, MAX_VELOCITY);
	rSpeedPid->setClamp(-MAX_VELOCITY, MAX_VELOCITY);
	// Also enable passthrough to not have any regulation
	lSpeedPid->setPassthrough(true);
	rSpeedPid->setPassthrough(true);
#else
	// All other robots use PWM
	lSpeedPid->setClamp(-DRIVER_DUTY_CLAMP, DRIVER_DUTY_CLAMP);
	rSpeedPid->setClamp(-DRIVER_DUTY_CLAMP, DRIVER_DUTY_CLAMP);
#endif

	// Setup PLLs
	PLL *lPll = new PLL(9.0f);
	PLL *rPll = new PLL(9.0f);

	// Setup accel limiters
	AccelLimiter *lSpeedAlim = new AccelLimiter(MAX_ACCEL);
	AccelLimiter *rSpeedAlim = new AccelLimiter(MAX_ACCEL);

	// Setup trapezoidal speed profile
	SpeedProfile *speedProfileDst = new SpeedProfile(MAX_LIN_VELOCITY, MAX_LIN_ACCEL);
	SpeedProfile *speedProfileAngle = new SpeedProfile(MAX_TURN_VELOCITY, MAX_TURN_ACCEL);

	// Setup the controller
	Controller *ctrl = new Controller(odo, speedProfileDst, speedProfileAngle, TOLERANCE_DST, TOLERANCE_ANGLE,
									MAX_LIN_ESTOP_ACCEL, MAX_TURN_ESTOP_ACCEL);

	// Finally setup the control loop, what will actually do all the processing
	ControlLoop *cl = new ControlLoop(lEnc, rEnc, lDrv, rDrv, odo,
									lSpeedPid, rSpeedPid, dstPid, anglePid, lPll, rPll, lSpeedAlim, rSpeedAlim,
									ctrl, ENCODER_WHEEL_RADIUS, POSITION_DOWNSAMPLING);

#ifdef ENABLE_EFFECTS
	Effects *effects = new Effects(cl, STOP_LIGHT_LEFT_PIN, BLINKER_LEFT_PIN, STOP_LIGHT_RIGHT_PIN, 
						BLINKER_RIGHT_PIN, STOP_LIGHT_CENTER_PIN, HEADLIGHT_LEFT_PIN, HEADLIGHT_RIGHT_PIN);
#endif

	// Init motor control
	multicore_launch_core1(comm_thread);

	// Send the ControlLoop ref over to the other core
	multicore_fifo_push_blocking((uint32_t)cl);
#ifdef ENABLE_EFFECTS
	multicore_fifo_push_blocking((uint32_t)effects);
#endif

	while (true) {
		absolute_time_t start = get_absolute_time();

		cl->work();
#ifdef ENABLE_EFFECTS
		effects->work();
#endif

		absolute_time_t end = get_absolute_time();

		// Try to keep the period 
		int64_t diff = absolute_time_diff_us(start, end);
		busy_wait_us(ASSERV_PERIOD_US-diff);
	}
}