#pragma once

#include <queue>
#include <cstring>

#include <pico/sync.h>

template <class T>
struct TelemPacket
{
	float timestamp;
	T data;
};

template <class T>
class Telemetry
{
public:
	Telemetry(size_t bufferSize) {
		this->bufferSize = bufferSize;
		mutex_init(&this->mutex);
		this->running = false;
		this->time = 0;
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

	void getInBuffer(uint8_t *buffer, size_t size) {
		if (size < 5)
			return;
		TelemPacket<T> elem = get();
		uint32_t sizeVal = this->size()&0xFFFF;
		memcpy(&buffer[0], &sizeVal, sizeof(uint32_t));
		memcpy(&buffer[4], &elem, sizeof(TelemPacket<T>));
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
		return 4 + sizeof(TelemPacket<T>);
	}

	void flush() {
		mutex_try_enter(&this->mutex, nullptr);
		std::queue<TelemPacket<T>> empty;
		std::swap(this->queue, empty);
		mutex_exit(&this->mutex);
	}
	
private:

	void addInner(T val, float ts) {
		TelemPacket<T> dat;
		dat.data = val;
		dat.timestamp = ts;
		if (size() >= this->bufferSize)
			this->queue.pop();
		this->queue.push(dat);
	}

	float time;

	bool running;
	
	size_t bufferSize;
	std::queue<TelemPacket<T>> queue;
	mutex_t mutex;
};