cmake_minimum_required(VERSION 3.13)

# Platform must be Pico for antenna tracker
set(PLATFORM "PICO" CACHE STRING "Target platform: PICO only")

# Set pico-sdk path
set(PICO_SDK_PATH "${CMAKE_SOURCE_DIR}/vcgimbal/lib/pico-sdk" CACHE PATH "Path to Pico SDK")

# Include pico SDK
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Set micro-ROS SDK path as environment variable for its CMakeLists
set(ENV{PICO_SDK_PATH} "${PICO_SDK_PATH}")
set(MICROROS_SDK_PATH "${CMAKE_SOURCE_DIR}/lib/micro_ros_raspberrypi_pico_sdk" CACHE PATH "Path to micro-ROS SDK")

project(vctracker VERSION 1.0.0 LANGUAGES CXX C)

# Initialize pico SDK
pico_sdk_init()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)

# Build target selection
set(BUILD_TARGET "main" CACHE STRING "Build target: main|test_gps|test_leds|test_gimbal|test_ros|system_test|all")

# Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -fexceptions")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -DDEBUG")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Build target: ${BUILD_TARGET}")
message(STATUS "Building Antenna Tracker for Raspberry Pi Pico with micro-ROS")

# Include directories
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/vcgimbal/include
    ${MICROROS_SDK_PATH}/libmicroros/include
)

# Link directories for prebuilt micro-ROS library
link_directories(${MICROROS_SDK_PATH}/libmicroros)

# Build vcgimbal library first
add_subdirectory(vcgimbal)

# Antenna tracker source files
set(TRACKER_SOURCES
    src/AntennaTracker.cpp
    src/GPSModule.cpp
    src/GeoCalculations.cpp
    src/StatusLED.cpp
    src/ROSParser.cpp
    src/pico_uart_transport.c
)

# Create antenna tracker library
add_library(antenna_tracker_lib STATIC ${TRACKER_SOURCES})

target_link_libraries(antenna_tracker_lib
    gimbal_lib
    pico_stdlib
    hardware_uart
    hardware_pwm
    hardware_clocks
    microros
)

target_compile_options(antenna_tracker_lib PRIVATE -fexceptions)

target_compile_definitions(antenna_tracker_lib PUBLIC PICO_BUILD=1)

# Set library output directory
set_target_properties(antenna_tracker_lib PROPERTIES
    OUTPUT_NAME antenna_tracker
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
)

# Helper macro to create executables
macro(create_executable TARGET_NAME SOURCE_FILE)
    add_executable(${TARGET_NAME} ${SOURCE_FILE})
    
    target_link_libraries(${TARGET_NAME}
        antenna_tracker_lib
        gimbal_lib
        pico_stdlib
        hardware_uart
        hardware_pwm
        hardware_clocks
        microros
    )
    
    pico_enable_stdio_usb(${TARGET_NAME} 1)
    pico_enable_stdio_uart(${TARGET_NAME} 0)
    pico_add_extra_outputs(${TARGET_NAME})
    
    set_target_properties(${TARGET_NAME} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    )
endmacro()

# Main antenna tracker executable
if(BUILD_TARGET STREQUAL "main" OR BUILD_TARGET STREQUAL "all")
    create_executable(antenna_tracker examples/main.cpp)
    message(STATUS "Building: Main antenna tracker application")
endif()

# GPS test - UBX protocol (works with M10G default settings)
if(BUILD_TARGET STREQUAL "test_gps" OR BUILD_TARGET STREQUAL "all")
    create_executable(test_gps examples/test_gps_ubx.cpp)
    message(STATUS "Building: GPS UBX protocol test")
endif()

# LED test
if(BUILD_TARGET STREQUAL "test_leds" OR BUILD_TARGET STREQUAL "all")
    create_executable(test_leds examples/test_leds.cpp)
    message(STATUS "Building: Status LED test")
endif()

# Gimbal test
if(BUILD_TARGET STREQUAL "test_gimbal" OR BUILD_TARGET STREQUAL "all")
    create_executable(test_gimbal examples/test_gimbal.cpp)
    message(STATUS "Building: Gimbal servo test")
endif()

# ROS test
if(BUILD_TARGET STREQUAL "test_ros" OR BUILD_TARGET STREQUAL "all")
    create_executable(test_ros examples/test_ros.cpp)
    message(STATUS "Building: micro-ROS integration test")
endif()

# Minimal ROS test
if(BUILD_TARGET STREQUAL "test_ros_minimal" OR BUILD_TARGET STREQUAL "all")
    add_executable(test_ros_minimal examples/test_ros_minimal.cpp)
    target_link_libraries(test_ros_minimal
        pico_stdlib
        hardware_gpio
        ${MICROROS_LIBRARY}
    )
    pico_add_extra_outputs(test_ros_minimal)
    message(STATUS "Building: Minimal micro-ROS test")
endif()

# System integration test
if(BUILD_TARGET STREQUAL "system_test" OR BUILD_TARGET STREQUAL "all")
    create_executable(system_test examples/system_test.cpp)
    message(STATUS "Building: Complete system integration test")
endif()

message(STATUS "=== Antenna Tracker Build Configuration ===")
message(STATUS "  Output: ${CMAKE_BINARY_DIR}/bin/")
message(STATUS "  USB Serial: Enabled")
message(STATUS "  micro-ROS: Enabled")

message(STATUS "  UART Serial: Disabled")
message(STATUS "  micro-ROS: Enabled")
message(STATUS "==========================================")
