cmake_minimum_required(VERSION 3.13)
project(baykael_serial_device)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_RPATH_USE_ORIGIN TRUE)

# Serial Device Library
add_library(baykael_serial_device SHARED src/SerialDevice.cpp)
target_include_directories(baykael_serial_device PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/serial_device>
)