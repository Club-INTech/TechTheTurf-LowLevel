#include <stdio.h>
#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <hardware/clocks.h>
#include <pico/rand.h>
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

//#define TEST_MUSIC
//#define TEST_COMM

//#define SERIAL_COMM

#ifndef TEST_MUSIC
#ifndef TEST_COMM

#ifdef SERIAL_COMM
volatile float serialDst, serialAngle;
volatile uint8_t serialCmd, serialSquare;

void comm_thread() {
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
			case 'c':
				scanf("%f", &serialDst);
				serialCmd = 4;
				serialSquare = 0;
				break;
			case 'r':
				serialCmd = 2;
				break;
			default:
				break;
		}
	}
}
#else

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

#endif

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
	PID *lSpeedPid = new PID(0.001f, 0.002f, 0.00005f);
	PID *rSpeedPid = new PID(0.001f, 0.002f, 0.00005f);
	PID *dstPid = new PID(10.0f, 0.5f, 0.05f/*, 1000.0f*/);
	PID *anglePid = new PID(1000.0f, 2.0f, 0.5f);

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

	uint nb = 0;

	multicore_launch_core1(comm_thread);

#ifndef SERIAL_COMM
	// Send the ControlLoop ref over to the other core
	multicore_fifo_push_blocking((uint32_t)cl);
#endif

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
#ifdef SERIAL_COMM
		if (serialCmd != 0) {
			if (serialCmd == 1)  {
				ctrl->movePolar(serialDst, (serialAngle / 180.0f) * M_PI);
				serialCmd = 0;
			} else if (serialCmd == 2) {
				odo->reset();
				ctrl->reset();
				dstPid->reset();
				anglePid->reset();
				serialCmd = 0;
				serialSquare = 0;
			} else if (serialCmd == 3) {
				ctrl->setTarget(serialDst, serialAngle);
				serialCmd = 0;
			} else if (serialCmd == 4) {
				if (ctrl->canQueueMove()) {
					if (serialSquare%2 == 0) {
						ctrl->movePolar(serialDst, 0.0f);
					} else if (serialSquare%2 == 1) {
						ctrl->movePolar(0.0f, ((-90.0f) / 180.0f) * M_PI);
					}
					serialSquare++;
				}
				if (serialSquare == 8)
					serialCmd = 0;
			}
		}
#endif
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

		//printf("Enc: l:%i r:%i\n", lEnc->getCount(), rEnc->getCount());

		// Try to keep the period 
		int64_t diff = absolute_time_diff_us(start, end);
		busy_wait_us(ASSERV_PERIOD_US-diff);
	}
}

#else

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

#endif

#else

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

uint16_t bestDivs[256] = {2558,1504,3326,1788,3809,3766,3070,2737,4090,2554,2802,233,2558,1717,3326,894,3809,3766,3070,2737,2927,1251,3310,233,509,396,1663,673,1955,3766,3070,2737,503,666,2784,233,509,2804,2879,673,1678,1883,3070,2947,848,333,955,1241,509,2022,1453,2384,839,172,1591,1080,2785,3324,1586,1241,509,3086,1453,1542,993,386,2843,540,2785,3324,793,928,509,1543,1453,771,993,386,3074,270,2000,3324,2052,928,509,2819,1453,1646,173,386,1537,2999,3,1662,1026,2792,509,4072,453,823,173,386,2816,761,2049,831,513,2792,509,3073,3244,1424,2134,386,333,761,3072,1280,2304,2792,3840,3584,1622,3352,1067,386,1352,761,1536,2793,1889,2792,2447,1792,811,1233,2581,1536,1600,761,768,3444,512,1396,2447,1909,3884,1710,3338,768,800,761,1882,1072,256,768,1413,3379,2442,3996,3830,3467,512,2428,512,536,3970,3884,1373,988,2442,964,1915,3889,256,3431,256,268,3970,1015,1513,988,3535,964,953,3246,4004,2971,3986,2954,3970,2427,2676,622,3943,354,2396,1623,2130,3661,2121,1605,1857,3389,1338,311,3891,305,1326,2859,1065,3878,3108,2850,3104,3614,797,2075,26,2072,535,3605,2580,1811,1554,1297,1552,1807,2318,3085,13,1036,2315,3850,1290,3081,777,2824,776,3079,1287,3590,2054,518,3333,1797,517,3588,2308,1284,260,3587,2563,1795,1027,259};
uint16_t bestRes[256] = {60060,64334,53468,50942,53724,62632,42448,57448,38382,36282,35366,34760,30030,39782,26734,50942,26862,31316,21224,28724,43118,20002,17970,17380,15104,25758,26734,19948,18562,15658,10612,14362,9746,14746,9550,8690,7552,7374,26734,9974,10654,15658,5306,9686,15014,14746,5730,4660,3776,3914,4912,9974,10654,4162,12202,11338,2668,2248,10646,2330,1888,30574,2456,63022,1684,2752,12202,11338,1334,1124,10646,1580,944,30574,1228,63022,842,1376,61426,11338,722,562,29794,790,472,30574,614,910,548,688,61426,434,25082,562,29794,272,236,242,270,910,274,344,61426,160,25082,562,29794,136,118,16106,154,174,274,172,274,80,25082,56818,29794,68,15926,16106,154,506,274,86,146,40,25082,38,86,34,52,16106,154,30,274,14914,82,20,25082,38,33518,34,26,30,74,18,274,14914,82,10,26,46,33518,10546,14,34,12,10,6,10,10558,10,9406,46,8,22,10,4,6,4,6,14,10558,6,9406,46,4,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2};

void setFreq(Driver *drv, float freq) {
	drv->setFreq(freq);
	return;
	uint besttop = 32;
	uint bestdiv = 1;
	float bestfreq = 1;
	float bestdiff = 1e9;
	float sysclk = (float)clock_get_hz(clk_sys);
	for (uint top=32; top<16384; top+=64) {
		for (uint div=1; div<256; div+=2) {
			float cfreq = sysclk/(top*div);
			float diff = abs(freq-cfreq);
			if (diff < bestdiff) {
				bestdiff = diff;
				besttop = top;
				bestfreq = cfreq;
				bestdiv = div;
			}
		}
	}
	printf("%f %f %f %i\n", bestdiff, freq, bestfreq, besttop);
	drv->setResolution(besttop);
	drv->setClkDiv(bestdiv, 0);
}

void startNote(int chan, int note, int vel) {
	int idx = getOpenChan();
	if (idx == -1)
		return;

	float freq = std::pow(2.0,(((float)note)-69.0)/12.0)*440.0f;
	printf("%i %f %i\n", idx, freq, note);

	channels[idx] = chan;
	notes[idx] = note;
	velocities[idx] = vel;
	
	setFreq(drivers[idx], freq);
	//drivers[idx]->setResolution(bestRes[note]);
	//drivers[idx]->setClkDiv(bestDivs[note]&0xFF, (bestDivs[note]>>8)&0xFF);
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
		//float rnd = ((float)get_rand_32())/4294967295.0; 
		//drivers[0]->setFreq(f+((rnd-0.5)*2.0f*50.0f));
		setFreq(drivers[0], f);
		drivers[0]->setPwm(0.1f);
		sleep_ms(10);
		printf("%f\n", f);
	}
	//for (float f=1e3; f>=0;f-=10) {
	//	float rnd = ((float)get_rand_32())/4294967295.0; 
	//	//drivers[0]->setFreq(f+((rnd-0.5)*2.0f*50.0f));
	//	drivers[0]->setFreq(f);
	//	drivers[0]->setPwm(0.5f);
	//	sleep_ms(10);
	//	printf("%f\n", f);
	//}

}

int main() {
	// Init PicoSDK
	stdio_init_all();
	sleep_ms(2000);
	printf("Hello %i\n", clock_get_hz(clk_sys));

	Driver *lDrv = new Driver(LEFT_MOTOR_FW_PIN, LEFT_MOTOR_RW_PIN, false, 8192, 40e3, 0.0);
	Driver *rDrv = new Driver(RIGHT_MOTOR_FW_PIN, RIGHT_MOTOR_RW_PIN, false, 8192, 40e3, 0.0);
	drivers[0] = rDrv;
	drivers[1] = lDrv;

	int chan, note, vel;
	float tmp1, tmp2;

	while (true) {
		switch (getchar()) {
			case 'f':
				scanf("%i %f %f", &chan, &tmp1, &tmp2);
				setFreq(drivers[chan], tmp1);
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
#endif