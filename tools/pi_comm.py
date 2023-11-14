from smbus2 import SMBus, i2c_msg, I2cFunc
import random

busId = 1
addr = 0x69

def send_dat(bus, data):
	write = i2c_msg.write(addr, data)
	bus.i2c_rdwr(write)

	read = i2c_msg.read(addr, size)
	bus.i2c_rdwr(read)
	return bytes(read)

for i in range(1000):
	with SMBus(busId) as bus:
		# Write a byte to address 80, offset 0
		size = random.randrange(1,64)
		data = bytes([random.randint(0,255) for i in range(size)])

		dat = send_dat(bus,data)
		print(data)
		print(dat)
		assert(dat == data)