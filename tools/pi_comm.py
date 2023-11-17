from smbus2 import SMBus, i2c_msg, I2cFunc
import random

busId = 1
addr = 0x69

def send_dat(bus, data, respSize = 0):
	write = i2c_msg.write(addr, data)
	bus.i2c_rdwr(write)

	if respSize > 0:
		read = i2c_msg.read(addr, respSize)
		bus.i2c_rdwr(read)
		return bytes(read)

for i in range(1000):
	with SMBus(busId) as bus:
		# Write a byte to address 80, offset 0
		if random.random() < 0.5:
			print("Norsp")
			size = random.randrange(1,60)
			data = bytes([0]+[random.randint(0,255) for i in range(size)])
			send_dat(bus, data)
		else:
			print("Rsp")
			size = random.randrange(1,60)
			data = bytes([1]+[random.randint(0,255) for i in range(size)])
			dat = send_dat(bus, data, len(data))
			assert(dat == data)