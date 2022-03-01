cmake_minimum_required(VERSION 3.8)
project(tests VERSION 0.1.0)

set(Boost_USE_STATIC_LIBS OFF)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(FastRTPS_INCLUDE_DIR /opt/ros/foxy/include) 
set(FastRTPS_LIBRARY_RELEASE /opt/ros/foxy/lib/libfastrtps.so)

find_package(Boost REQUIRED COMPONENTS unit_test_framework system thread regex)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(oxts_msgs REQUIRED)

message(STATUS "current source dir: ${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(tests
               tests.cpp
               oxts_driver.cpp
               udp_server_client.cpp
               convert.cpp
               nav_conversions.cpp
               wrapper.cpp
               ../oxts_driver/src/driver/driver.cpp
               ../oxts_driver/src/lib/NComRxC.c
               ../oxts_ins/src/conversions/convert.cpp
               ../oxts_ins/src/conversions/wrapper.cpp
               ../oxts_ins/src/lib/nav_conversions.cpp)

ament_target_dependencies(tests
        rclcpp
        std_msgs
        sensor_msgs
        geometry_msgs
        nav_msgs
        tf2
        tf2_kdl
        oxts_msgs)

add_custom_command(
        TARGET tests POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
                ${CMAKE_SOURCE_DIR}/test.ncom
                ${CMAKE_CURRENT_BINARY_DIR}/test.ncom)

message(STATUS "include dirs: ${Boost_INCLUDE_DIRS}")

target_include_directories(tests PUBLIC
        ../oxts_driver/include
        ../oxts_ins/include
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        PRIVATE ${Boost_INCLUDE_DIRS}
        $<INSTALL_INTERFACE:include>)

target_link_libraries (tests 
        ${Boost_LIBRARIES}
        ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
