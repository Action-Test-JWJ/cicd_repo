cmake_minimum_required(VERSION 3.8)
project(aeirobot_hand_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Check robot type and skip build if necessary
set(ROBOT_TYPE "edie" CACHE STRING "Select robot type: edie or alice")
if(ROBOT_TYPE STREQUAL "edie")
  message(STATUS "Skipping package aeirobot_hand_msgs for ROBOT_TYPE=${ROBOT_TYPE}")
  return()
endif()

find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Setangle.srv"
  "srv/Getangleact.srv"
  "srv/Getangleset.srv"
  "srv/Setpos.srv"
  "srv/Getposact.srv"
  "srv/Getposset.srv"
  "srv/Setspeed.srv"
  "srv/Getspeedset.srv"
  "srv/Setforce.srv"
  "srv/Getforceact.srv"
  "srv/Getforceset.srv"
  "srv/Getcurrentact.srv"
  "srv/Geterror.srv"
  "srv/Gettemp.srv"
  "msg/GetAngleAct.msg"
  "msg/GetAngleSet.msg"
  "msg/GetCurrentAct.msg"
  "msg/GetError.msg"
  "msg/GetForceAct.msg"
  "msg/GetForceSet.msg"
  "msg/GetPosAct.msg"
  "msg/GetPosSet.msg"
  "msg/GetSpeedSet.msg"
  "msg/GetTemp.msg"
  "msg/HandTask.msg"
  "msg/SetAngle.msg"
  "msg/SetAngleResult.msg"
  "msg/SetForce.msg"
  "msg/SetForceResult.msg"
  "msg/SetPos.msg"
  "msg/SetPosResult.msg"
  "msg/SetSpeed.msg"
  "msg/SetSpeedResult.msg"


  DEPENDENCIES 
 )
 
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
