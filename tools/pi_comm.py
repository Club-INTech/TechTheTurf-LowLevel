from smbus2 import SMBus, i2c_msg, I2cFunc
import random

busId = 1
addr = 0x69

with SMBus(busId) as bus:
	# Write a byte to address 80, offset 0
	size = 8
	data = bytes([random.randint(0,255) for i in range(size)])

	print(data,size)

	write = i2c_msg.write(addr, data)
	read = i2c_msg.read(addr, len(data))
	bus.i2c_rdwr(write, read)
	dat = bytes(read)
	print(data, dat)