cmake_minimum_required(VERSION 3.8)
project(oxts_tests VERSION 0.1.0)

set(Boost_USE_STATIC_LIBS OFF)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Boost REQUIRED COMPONENTS unit_test_framework)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(oxts_driver REQUIRED)
find_package(oxts_ins REQUIRED)
find_package(oxts_msgs REQUIRED)

add_executable(tests
  tests.cpp
  oxts_driver.cpp
  udp_server_client.cpp
  convert.cpp
  nav_conversions.cpp
  wrapper.cpp)

ament_target_dependencies(tests
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_kdl
  oxts_driver
  oxts_ins
  oxts_msgs)

target_include_directories(tests PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PRIVATE ${Boost_INCLUDE_DIRS}
  $<INSTALL_INTERFACE:include>)
target_link_libraries (tests ${Boost_LIBRARIES})

install(TARGETS tests
  DESTINATION lib/${PROJECT_NAME})

ament_package()
