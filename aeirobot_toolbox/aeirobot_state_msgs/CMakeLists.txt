cmake_minimum_required(VERSION 3.8)
project(aeirobot_state_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Check robot type and skip build if necessary
set(ROBOT_TYPE "edie" CACHE STRING "Select robot type: edie or alice")
if(ROBOT_TYPE STREQUAL "edie")
  message(STATUS "Skipping package aeirobot_state_msgs for ROBOT_TYPE=${ROBOT_TYPE}")
  return()
endif()

find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EndpointState.msg"
  "msg/FsrState.msg"
  "msg/RobotState.msg"
  "msg/RobotGoal.msg"
  "msg/Sensors.msg"
  "msg/StateEstimator.msg"
  "msg/WholeBodyLog.msg"
  DEPENDENCIES
  builtin_interfaces
  std_msgs
  sensor_msgs
  geometry_msgs
 )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
