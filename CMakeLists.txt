cmake_minimum_required(VERSION 3.13)
project(pendarlab_serial_device)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_RPATH_USE_ORIGIN TRUE)

find_package(GTest CONFIG REQUIRED)

# Serial Device Library
add_library(pendarlab_serial_device SHARED src/SerialDevice.cpp)
target_include_directories(pendarlab_serial_device PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/serial_device>
)

# Test
add_executable(serial_device_test test/serial_device_test.cpp)
target_include_directories(serial_device_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(serial_device_test PUBLIC pendarlab_serial_device gtest)