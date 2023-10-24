#include <stdio.h>
#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <pico/rand.h>

#include <comm.hpp>
#include <encoder.hpp>
#include <driver.hpp>
#include <control_loop.hpp>
#include <pll.hpp>
#include <accel_limiter.hpp>
#include <pins.hpp>
#include <math.h>


volatile float serialDst, serialAngle;
volatile uint8_t serialCmd;

void serial_proc() {
	while (true) {
		switch (getchar()) {
			case 'a':
				serialDst = 0.0f;
				scanf("%f", &serialAngle);
				serialCmd = 1;
				break;
			case 'd':
				serialAngle = 0.0f;
				scanf("%f", &serialDst);
				serialCmd = 1;
				break;
			case 's':
				scanf("%f %f", &serialDst, &serialAngle);
				serialCmd = 3;
				break;
			case 'r':
				serialCmd = 2;
				break;
			default:
				break;
		}
	}
}

#define ASSERV_PERIOD_US 2000

int main() {
	// Init PicoSDK
	stdio_init_all();
	//sleep_ms(2000);
	//printf("Hello\n");

	// Init HL Comms
	//Comm *hlComm = new Comm(I2C_SDA, I2C_SCL, 0x69, i2c0);

	// Init encoders & drivers
	Encoder *lEnc = new Encoder(LEFT_INCREMENTAL_A_PIN, LEFT_INCREMENTAL_B_PIN, true, 0);
	Encoder *rEnc = new Encoder(RIGHT_INCREMENTAL_A_PIN, RIGHT_INCREMENTAL_B_PIN, true, 1);
	Driver *lDrv = new Driver(LEFT_MOTOR_FW_PIN, LEFT_MOTOR_RW_PIN, false);
	Driver *rDrv = new Driver(RIGHT_MOTOR_FW_PIN, RIGHT_MOTOR_RW_PIN, false);

	// Init odometry
	Odometry *odo = new Odometry(86.8f/2.0f);

	// Setup PIDs
	PID *lSpeedPid = new PID(0.001f, 0.001f, 0.0f);
	PID *rSpeedPid = new PID(0.001f, 0.001f, 0.0f);
	PID *dstPid = new PID(10.0f, 0.0f, 0.0f);
	PID *anglePid = new PID(800.0f, 0.0f, 0.0f);

	// Setup PLLs
	PLL *lPll = new PLL(9.0f);
	PLL *rPll = new PLL(9.0f);

	// Setup accel limiters
	AccelLimiter *lAlim = new AccelLimiter(1000.0f);
	AccelLimiter *rAlim = new AccelLimiter(1000.0f);

	// Setup the controller
	Controller *ctrl = new Controller(odo);

	ControlLoop *cl = new ControlLoop(lEnc, rEnc, lDrv, rDrv, odo,
									lSpeedPid, rSpeedPid, dstPid, anglePid, lPll, rPll, lAlim, rAlim,
									ctrl, 34.0f/2.0f);

	// Init motor control
	//printf("Begin\n");

	uint nb = 0;

	multicore_launch_core1(serial_proc);

	while (true) {
		absolute_time_t start = get_absolute_time();
		if (serialCmd != 0) {
			if (serialCmd == 1)  {
				ctrl->movePolar(serialDst, (serialAngle / 180.0f) * M_PI);
			} else if (serialCmd == 2) {
				odo->reset();
				ctrl->reset();
				dstPid->reset();
				anglePid->reset();
			} else if (serialCmd == 3) {
				ctrl->setTarget(serialDst, serialAngle);	
			}
			serialCmd = 0;
		}
		//printf("%i %f\n", nb, (((float)std::min(nb,1000u))/1000.0f)*1.0f);
		//if (nb == 200)
		//	ctrl->movePolar(-100.0f,0);
		//else if (nb == 2000)
		//	ctrl->movePolar(-100.0f,0);
		//if (nb == 200)
		//	ctrl->movePolar(50.0f, 0);
		//if (nb == 400)
		//	ctrl->movePolar(-50.0f, 0);
		//ctrl->targetDst = (((float)std::min(nb,250u))/250.0f)*500.0f + -500.0f * (((float)std::min(std::max(((int)nb)-(500*4),0),250))/250.0f);
		cl->work();
		nb++;
		absolute_time_t end = get_absolute_time();

		// Try to keep the period 
		int64_t diff = absolute_time_diff_us(start, end);
		busy_wait_us(ASSERV_PERIOD_US-diff);
	}
}

/*
// Comm test
int main() {
	// Init PicoSDK
	stdio_init_all();
	//sleep_ms(2000);
	printf("Hello\n");

	// Init HL Comms
	Comm *hlComm = new Comm(I2C_SDA, I2C_SCL, 0x69, i2c0);

	while (true) {
		busy_wait_us(1000000);
	}
}
*/
/*
// Music test
const int channelNb = 2;
Driver *drivers[channelNb] = {nullptr, nullptr};
int channels[channelNb] = {-1, -1};
int notes[channelNb] = {-1, -1};
int velocities[channelNb] = {0, 0};

int getOpenChan() {
	for (int i=0;i<channelNb;i++) {
		if (channels[i] == -1)
			return i;
	}
	return -1;
}

int findChan(int chan, int note) {
	for (int i=0;i<channelNb;i++) {
		if (channels[i] == chan && notes[i] == note)
			return i;
	}
	return -1;
}

void startNote(int chan, int note, int vel) {
	int idx = getOpenChan();
	if (idx == -1)
		return;

	float freq = std::pow(2.0,(((float)note)+(12.0*0)-69.0)/12.0)*440.0f;
	//printf("%f %i\n", freq, note);

	channels[idx] = chan;
	notes[idx] = note;
	velocities[idx] = vel;
	drivers[idx]->setFreq(freq);
	//0.15 * (((float)vel)/127.0)
	drivers[idx]->setPwm(0.5f);
}

void stopNoteChan(int dChan) {
	drivers[dChan]->setPwm(0);
	channels[dChan] = -1;
	notes[dChan] = -1;
	velocities[dChan] = 0;
}

void stopNote(int chan, int note, int vel) {
	int idx = findChan(chan, note);
	if (idx == -1)
		return;
	stopNoteChan(idx);
}

void stopAllNotes() {
	for (int i=0;i<channelNb;i++)
		stopNoteChan(i);
}

void sweep() {
	for (float f=0; f<=1e3;f+=10) {
		float rnd = ((float)get_rand_32())/4294967295.0; 
		//drivers[0]->setFreq(f+((rnd-0.5)*2.0f*50.0f));
		drivers[0]->setFreq(f);
		drivers[0]->setPwm(0.5f);
		sleep_ms(10);
		printf("%f\n", f);
	}
	for (float f=1e3; f>=0;f-=10) {
		float rnd = ((float)get_rand_32())/4294967295.0; 
		//drivers[0]->setFreq(f+((rnd-0.5)*2.0f*50.0f));
		drivers[0]->setFreq(f);
		drivers[0]->setPwm(0.5f);
		sleep_ms(10);
		printf("%f\n", f);
	}

}

int main() {
	// Init PicoSDK
	stdio_init_all();
	sleep_ms(2000);
	printf("Hello\n");

	Driver *lDrv = new Driver(LEFT_MOTOR_FW_PIN, 1024, 40e3, 0.0);
	Driver *rDrv = new Driver(RIGHT_MOTOR_FW_PIN, 1024, 40e3, 0.0);
	drivers[0] = lDrv;
	drivers[1] = rDrv;

	int chan, note, vel;
	float tmp1, tmp2;

	while (true) {
		switch (getchar()) {
			case 'f':
				scanf("%i %f %f", &chan, &tmp1, &tmp2);
				drivers[chan]->setFreq(tmp1);
				drivers[chan]->setPwm(tmp2);
				printf("%i %f %f\n", chan, tmp1, tmp2);
				break;
			case 's':
				sweep();
				break;
			case 'b':
				scanf("%i,%i,%i", &chan, &note, &vel);
				startNote(chan, note, vel);
				break;
			case 'e':
				scanf("%i,%i,%i", &chan, &note, &vel);
				stopNote(chan, note, vel);
				break;
			case 'r':
				stopAllNotes();
				break;
			default:
				break;
		}
	}
}
*/