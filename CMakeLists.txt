cmake_minimum_required(VERSION 3.13)

include(/home/intech/pico/pico-sdk/pico_sdk_init.cmake)

project(pami)

pico_sdk_init()


add_executable(pami-main
    src/main.c
    src/motor.c
    src/servo.c
    src/incremental_encoder.c
)

pico_enable_stdio_usb(pami-main 1)
pico_enable_stdio_uart(pami-main 0)

target_link_libraries(pami-main pico_stdlib hardware_pwm)
pico_add_extra_outputs(pami-main)

