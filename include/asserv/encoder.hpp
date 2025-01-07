#pragma once

#include <hardware/pio.h>

class Encoder
{
public:
	Encoder(uint pin_a, uint pin_b, bool reversed=false, uint state_machine=0, PIO pio=pio0);
	~Encoder();

	void update();
	int32_t getCount();
	int32_t getSpeedCount();
	float convertRevolutions(int32_t ticks);
	float getRevolutions();
	float getSpeedRev();
	void reset(int32_t cnt=0);

	void calibratePhase();
	void setCalibrationData(int step0, int step1, int step2);

private:
	void pioInit();
	void pioCleanup();

	void fetchCounts(uint *step, int *cycles, uint *us);
	void readPioData(uint *step, uint *step_us, uint *transition_us, int *forward);
	uint getStepStartTransitionPos(uint step);


	uint calibration_data[4]; // relative phase sizes
	uint clocks_per_us;       // save the clk_sys frequency in clocks per us
	uint idle_stop_samples;   // after these samples without transitions, assume the encoder is stopped

	// internal fields to keep track of the previous state:
	uint prev_trans_pos, prev_trans_us;
	uint prev_step_us;
	uint prev_low, prev_high;
	uint idle_stop_sample_count;
	int speed_2_20;
	int stopped;

	// output of the encoder update function:
	int speed;     // estimated speed in substeps per second
	uint position; // estimated position in substeps

	uint raw_step; // raw step count

	PIO pio;

	bool reversed;
	uint pinAB;
	uint stateMachine;
};
