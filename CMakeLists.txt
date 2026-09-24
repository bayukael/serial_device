cmake_minimum_required(VERSION 3.13)
project(pendarlab_serial_device VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_RPATH_USE_ORIGIN TRUE)
set(NAMESPACE pendarlab)
set(PACKAGE_FILENAME serial_device)
set(PACKAGE_NAME PendarlabSerialDevice)

string(REGEX REPLACE "/+$" "" _bt_base "${CMAKE_INSTALL_PREFIX}")
if(NOT _bt_base MATCHES "/${PROJECT_VERSION}$")
    set(CMAKE_INSTALL_PREFIX "${_bt_base}/${PROJECT_VERSION}" CACHE PATH "" FORCE)
endif()

find_package(GTest CONFIG REQUIRED)

# Serial Device Library
add_library(SerialDevice SHARED src/SerialDevice.cpp)

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/include/${PACKAGE_FILENAME}/Version.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/include/${PACKAGE_FILENAME}/Version.h
    @ONLY
)

target_include_directories(SerialDevice PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
add_library(${NAMESPACE}::SerialDevice ALIAS SerialDevice)
set_target_properties(SerialDevice PROPERTIES
    OUTPUT_NAME pendarlab-serial_device
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)

# Test
add_executable(serial_device_test test/serial_device_test.cpp)
target_include_directories(serial_device_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(serial_device_test PUBLIC SerialDevice gtest)

##############################################################################
######################     Define Files to Install     #######################
##############################################################################

set(TARGET_EXPORT_NAME SerialDeviceTargets)
set(TARGET_EXPORT_FILENAME serial_device_targets)
set(PACKAGE_DIRECTORY_DESTINATION lib/cmake/${PACKAGE_NAME})

install(
    TARGETS SerialDevice
    EXPORT ${TARGET_EXPORT_NAME}
)
install(
    DIRECTORY include/
    DESTINATION include
    PATTERN "*.in" EXCLUDE
)
install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/include/${PACKAGE_FILENAME}/Version.h
    DESTINATION include/${PACKAGE_FILENAME}
)
install(
    EXPORT ${TARGET_EXPORT_NAME}
    FILE ${TARGET_EXPORT_FILENAME}.cmake
    NAMESPACE ${NAMESPACE}::
    DESTINATION ${PACKAGE_DIRECTORY_DESTINATION}
)

##############################################################################
##########################     CMake Packaging     ###########################
##############################################################################

include(CMakePackageConfigHelpers)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PACKAGE_FILENAME}-config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PACKAGE_NAME}Config.cmake
    INSTALL_DESTINATION ${PACKAGE_DIRECTORY_DESTINATION}
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/${PACKAGE_NAME}ConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PACKAGE_NAME}Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${PACKAGE_NAME}ConfigVersion.cmake
    DESTINATION ${PACKAGE_DIRECTORY_DESTINATION}
)
