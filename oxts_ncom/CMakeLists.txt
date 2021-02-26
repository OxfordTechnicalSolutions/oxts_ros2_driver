cmake_minimum_required(VERSION 3.8)
project(oxts_ncom VERSION 0.1.0)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
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


add_executable(oxts_ncom 
               src/ncom/ncom.cpp 
               src/ncom/ncom_node.cpp
               src/ncom/wrapper.cpp
               src/lib/NComRxC.c
               src/lib/convert.cpp)

ament_target_dependencies(oxts_ncom 
                          rclcpp
                          std_msgs
                          sensor_msgs
                          geometry_msgs
                          nav_msgs
                          tf2
                          tf2_kdl
                          oxts_msgs)
target_include_directories(oxts_ncom PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PRIVATE ${Boost_INCLUDE_DIRS}
  $<INSTALL_INTERFACE:include>)


install(TARGETS oxts_ncom
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
        config urdf rviz
        DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
