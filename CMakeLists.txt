cmake_minimum_required(VERSION 3.12)

# PicoSDK Include
if (DEFINED ENV{PICO_SDK_PATH} AND (NOT PICO_SDK_PATH))
    set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
    message("Using PICO_SDK_PATH from environment ('${PICO_SDK_PATH}')")
endif ()

set(PICO_SDK_PATH "${PICO_SDK_PATH}" CACHE PATH "Path to the Raspberry Pi Pico SDK")

get_filename_component(PICO_SDK_PATH "${PICO_SDK_PATH}" REALPATH BASE_DIR "${CMAKE_BINARY_DIR}")
if (NOT EXISTS ${PICO_SDK_PATH})
    message(FATAL_ERROR "Directory '${PICO_SDK_PATH}' not found")
endif ()

set(PICO_SDK_INIT_CMAKE_FILE ${PICO_SDK_PATH}/pico_sdk_init.cmake)
if (NOT EXISTS ${PICO_SDK_INIT_CMAKE_FILE})
    message(FATAL_ERROR "Directory '${PICO_SDK_PATH}' does not appear to contain the Raspberry Pi Pico SDK")
endif ()

set(PICO_SDK_PATH ${PICO_SDK_PATH} CACHE PATH "Path to the Raspberry Pi Pico SDK" FORCE)

include(${PICO_SDK_INIT_CMAKE_FILE})
# End PicoSDK

project(pami C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        )

if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wno-maybe-uninitialized)
endif()

add_executable(pami
            src/main.cpp
            src/encoder.cpp src/driver.cpp
            src/control_loop.cpp src/pid.cpp src/odometry.cpp src/controller.cpp src/pll.cpp src/accel_limiter.cpp
            src/comm.cpp)

include_directories(include/)

pico_enable_stdio_usb(pami 1)
pico_enable_stdio_uart(pami 0)

pico_generate_pio_header(pami ${CMAKE_CURRENT_LIST_DIR}/pio/quadrature_encoder.pio)

# pull in common dependencies
target_link_libraries(pami pico_i2c_slave pico_stdlib pico_multicore pico_sync pico_rand hardware_pio hardware_i2c hardware_pwm)

# create map/bin/hex file etc.
pico_add_extra_outputs(pami)