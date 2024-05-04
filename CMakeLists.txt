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

project(ll C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

# Include dirs
include_directories(include/)

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        )

if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wno-maybe-uninitialized)
endif()

# Shared library to all pico FW
add_library(ll_common INTERFACE)
target_sources(ll_common INTERFACE  "${CMAKE_CURRENT_SOURCE_DIR}/src/shared/comm.cpp")

# Linked libs
target_link_libraries(ll_common INTERFACE pico_i2c_slave pico_stdlib pico_multicore pico_sync hardware_i2c)

# Shared library to all pico asserv FW
add_library(ll_common_asserv INTERFACE)
target_sources(ll_common_asserv INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/src/asserv/comm_asserv.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/asserv/encoder.cpp"
                            "${CMAKE_CURRENT_SOURCE_DIR}/src/asserv/control_loop.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/asserv/pid.cpp"
                            "${CMAKE_CURRENT_SOURCE_DIR}/src/asserv/odometry.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/asserv/controller.cpp"
                            "${CMAKE_CURRENT_SOURCE_DIR}/src/asserv/pll.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/asserv/accel_limiter.cpp"
                            "${CMAKE_CURRENT_SOURCE_DIR}/src/asserv/speed_profile.cpp")

# Linked Libs
target_link_libraries(ll_common_asserv INTERFACE ll_common hardware_pwm hardware_pio)

# PIO header gen.
pico_generate_pio_header(ll_common_asserv ${CMAKE_CURRENT_LIST_DIR}/pio/quadrature_encoder.pio)

# Shared library to all pico asserv FW
add_library(dynamixel_sdk INTERFACE)
target_sources(dynamixel_sdk INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/src/dynamixel_sdk/group_bulk_read.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/dynamixel_sdk/group_bulk_write.cpp"
                                        "${CMAKE_CURRENT_SOURCE_DIR}/src/dynamixel_sdk/group_fast_bulk_read.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/dynamixel_sdk/group_fast_sync_read.cpp"
                                        "${CMAKE_CURRENT_SOURCE_DIR}/src/dynamixel_sdk/group_handler.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/dynamixel_sdk/group_sync_read.cpp"
                                        "${CMAKE_CURRENT_SOURCE_DIR}/src/dynamixel_sdk/group_sync_write.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/dynamixel_sdk/packet_handler.cpp"
                                        "${CMAKE_CURRENT_SOURCE_DIR}/src/dynamixel_sdk/port_handler.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/dynamixel_sdk/port_handler_pico.cpp"
                                        "${CMAKE_CURRENT_SOURCE_DIR}/src/dynamixel_sdk/protocol1_packet_handler.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/dynamixel_sdk/protocol2_packet_handler.cpp")

target_compile_definitions(dynamixel_sdk INTERFACE)

# Linked Libs
target_link_libraries(dynamixel_sdk INTERFACE pico_stdlib hardware_uart)

# Define Final interface libraries from which to make executables
# Asserv
# Paminable
add_library(ll_asserv_paminable INTERFACE)
target_sources(ll_asserv_paminable INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/src/asserv/driver.cpp")
target_compile_definitions(ll_asserv_paminable INTERFACE ASSERV ROBOT_PAMI PAMINABLE)
target_link_libraries(ll_asserv_paminable INTERFACE ll_common_asserv)

# Pamisérable
add_library(ll_asserv_pamiserable INTERFACE)
target_sources(ll_asserv_pamiserable INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/src/asserv/driver.cpp")
target_compile_definitions(ll_asserv_pamiserable INTERFACE ASSERV ROBOT_PAMI)
target_link_libraries(ll_asserv_pamiserable INTERFACE ll_common_asserv)

# Main robot
add_library(ll_asserv_main INTERFACE)
target_sources(ll_asserv_main INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/src/asserv/comm_bg.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/asserv/driver_bg.cpp"
                                        "${CMAKE_CURRENT_SOURCE_DIR}/src/asserv/comm_odrive.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/asserv/driver_odrive.cpp")
target_compile_definitions(ll_asserv_main INTERFACE ASSERV ROBOT_MAIN)
target_link_libraries(ll_asserv_main INTERFACE ll_common_asserv)

# Actuators
# Main robot
add_library(ll_action_main INTERFACE)
target_sources(ll_action_main INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/src/action/comm_action.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/action/dynamixel_manager.cpp"
                                        "${CMAKE_CURRENT_SOURCE_DIR}/src/action/dynamixel_motor.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/action/dynamixel_xl430.cpp"
                                        "${CMAKE_CURRENT_SOURCE_DIR}/src/action/stepper_driver.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/action/endstop.cpp"
                                        "${CMAKE_CURRENT_SOURCE_DIR}/src/action/elevator.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/action/pump.cpp"
                                        "${CMAKE_CURRENT_SOURCE_DIR}/src/action/arm.cpp")
target_compile_definitions(ll_action_main INTERFACE ROBOT_MAIN)
target_link_libraries(ll_action_main INTERFACE ll_common dynamixel_sdk)

# Pamigami
add_library(ll_pamigami INTERFACE)
target_sources(ll_pamigami INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/src/action/dynamixel_manager.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/action/dynamixel_motor.cpp"
                                     "${CMAKE_CURRENT_SOURCE_DIR}/src/action/dynamixel_xl430.cpp")
target_compile_definitions(ll_pamigami INTERFACE ROBOT_PAMI PAMIGAMI)
target_link_libraries(ll_pamigami INTERFACE ll_common dynamixel_sdk)

# Create executables from interface libs

add_executable(paminable "${CMAKE_CURRENT_SOURCE_DIR}/src/asserv/main.cpp")
target_link_libraries(paminable PUBLIC ll_asserv_paminable)

# Common to all projects
pico_enable_stdio_usb(paminable 1)
pico_enable_stdio_uart(paminable 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(paminable)

add_executable(pamiserable "${CMAKE_CURRENT_SOURCE_DIR}/src/asserv/main.cpp")
target_link_libraries(pamiserable PUBLIC ll_asserv_pamiserable)

# Common to all projects
pico_enable_stdio_usb(pamiserable 1)
pico_enable_stdio_uart(pamiserable 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(pamiserable)

add_executable(main_asserv "${CMAKE_CURRENT_SOURCE_DIR}/src/asserv/main.cpp")
target_link_libraries(main_asserv PUBLIC ll_asserv_main)

# Common to all projects
pico_enable_stdio_usb(main_asserv 1)
pico_enable_stdio_uart(main_asserv 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(main_asserv)

add_executable(main_action "${CMAKE_CURRENT_SOURCE_DIR}/src/action/main.cpp")
target_link_libraries(main_action PUBLIC ll_action_main)

# Common to all projects
pico_enable_stdio_usb(main_action 1)
pico_enable_stdio_uart(main_action 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(main_action)

add_executable(pamigami "${CMAKE_CURRENT_SOURCE_DIR}/src/action/pamigami.cpp")
target_link_libraries(pamigami PUBLIC ll_pamigami)

# Common to all projects
pico_enable_stdio_usb(pamigami 1)
pico_enable_stdio_uart(pamigami 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(pamigami)