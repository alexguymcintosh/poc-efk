cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_BOARD pico2 CACHE STRING "Board type")
set(PICO_PLATFORM rp2350 CACHE STRING "Platform")

include(pico_sdk_import.cmake)

project(imu C CXX ASM)

pico_sdk_init()

add_executable(imu main.cpp)

target_link_libraries(imu
    pico_stdlib
    hardware_i2c
)

pico_enable_stdio_usb(imu 1)
pico_enable_stdio_uart(imu 0)

pico_add_extra_outputs(imu)
