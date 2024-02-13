#pragma once

#include <queue>
#include <cstring>

#include <pico/sync.h>

class TelemetryBase
{
public:
	virtual bool isRunning() = 0;
	virtual void start() = 0;
	virtual void stop() = 0;
	virtual void getInBuffer(uint8_t *buffer, size_t size, uint8_t telemIdx) = 0;
	virtual size_t size() = 0;
	virtual size_t elemSize() = 0;
	virtual void setDownsample(uint8_t ds) = 0;
	virtual void flush() = 0;
};

template <class T>
struct TelemPacket
{
	float timestamp;
	T data;
};

template <class T, size_t bufferSize>
class Telemetry : public TelemetryBase
{
public:
	Telemetry() {
		mutex_init(&this->mutex);
		this->running = false;
		this->time = 0;
		this->downsample = 4;
		this->idx = 0;
		stop();
	}

	~Telemetry() {
		stop();
	}

	bool isRunning() {
		return this->running;
	}

	void start() {
		if (this->running)
			return;
		this->time = 0;
		this->running = true;
	}

	void stop() {
		if (!this->running)
			return;
		this->running = false;
		flush();
	}

	void add(T val, float dt) {
		if (!this->running)
			return;
		mutex_try_enter(&this->mutex, nullptr);
		this->time += dt;
		addInner(val, this->time);
		mutex_exit(&this->mutex);
	}

	void addAbs(T val, float time) {
		if (!this->running)
			return;
		mutex_try_enter(&this->mutex, nullptr);
		addInner(val, time);
		mutex_exit(&this->mutex);
	}

	void getInBuffer(uint8_t *buffer, size_t size, uint8_t telemIdx) {
		if (size < 3)
			return;
		TelemPacket<T> elem = get();
		buffer[0] = sizeof(TelemPacket<T>);
		buffer[1] = telemIdx;
		memcpy(&buffer[2], &elem, sizeof(TelemPacket<T>));
	}

	TelemPacket<T> get() {
		mutex_try_enter(&this->mutex, nullptr);
		TelemPacket<T> elem = this->queue.front();
		this->queue.pop();
		mutex_exit(&this->mutex);
		return elem;
	}

	size_t size() {
		return this->queue.size();
	}

	size_t elemSize() {
		return 2 + sizeof(TelemPacket<T>);
	}

	void setDownsample(uint8_t ds) {
		this->downsample = ds;
	}

	void flush() {
		mutex_try_enter(&this->mutex, nullptr);
		std::queue<TelemPacket<T>> empty;
		std::swap(this->queue, empty);
		mutex_exit(&this->mutex);
	}
	
private:

	void addInner(T val, float ts) {
		if (this->downsample > 0) {
			if (this->idx++ == this->downsample) {
				this->idx = 0;
			} else {
				return;
			}
		}
		TelemPacket<T> dat;
		dat.data = val;
		dat.timestamp = ts;
		if (size() >= bufferSize)
			this->queue.pop();
		this->queue.push(dat);
	}

	float time;

	bool running;
	uint8_t downsample;
	uint8_t idx;
	
	std::queue<TelemPacket<T>> queue;
	mutex_t mutex;
};