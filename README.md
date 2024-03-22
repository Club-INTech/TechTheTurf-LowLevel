# Low Level code of the PAMIsérable team for the CDR 2024

This is what is actually running on all the picos.

## What's inside ?

- `bg/` The PlatformIO project for the B-G431B-ESC1 using SimpleFOC.
- `src/asserv` The Pico SDK code for the Pico controlling the motors.
- `src/action` The Pico SDK code for the Pico controlling the actuators.
- `src/dynamixel_sdk` The Dynamixel SDK with a Pico SDK Port Handler implemented.
- `src/shared` Code shared across the motors and acuators controllers.
- `include/` Headers arranged in the same fashion as the `src/` folder
- `pio/` Code for the PIOs in the Picos.

## What does it compile into

- In the `bg/` PlatformIO project, there are 2 enviroments:
	- `left` for the left BG.
	- `right` for the right BG.

- In the Pico SDK cmake project, there are 4 executables compiled:
	- `main_asserv` for the main robot's motor controller pico.
	- `main_action` for the main robot's actuators controller pico.
	- `paminable` for the PAMInable pico.
	- `pamiserable` for the PAMIsérable pico.

## How do you compile it ?

- For the `bg/` PlatformIO project:
	- You might want to tweak sensor calibration values inside `bg/src/main.cpp` before using it.
	- You can either install the PlatformIO IDE and use it to build
	- Or build it using the command line with PlatformIO Core:
		- To do so, install [PlatformIO Core](https://docs.platformio.org/en/stable/core/installation/methods/index.html).
		- Then inside `bg/`, run `pio run` to compile both enviroments.
		- Use `pio run -e left -t upload` to upload the left bg code.
		- Use `pio run -e right -t upload` to upload the right bg code.

- For the Pico SDK cmake project
	- Install the [Pico SDK](https://github.com/raspberrypi/pico-sdk), make sure `PICO_SDK_PATH` is set
	- ```bash
		mkdir build
		cd build
		cmake ..
		make
		```
	- You will find the executables listed previously in the `build/` folder