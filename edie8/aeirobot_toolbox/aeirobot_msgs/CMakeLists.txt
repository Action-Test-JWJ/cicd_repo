cmake_minimum_required(VERSION 3.5)
project(aeirobot_msgs)

# Add C++ standard
set(CMAKE_CXX_STANDARD 17)

################################################################################
# Dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
)

################################################################################
# Declare ROS messages, services, and actions
################################################################################
set(msg_files
  "msg/PointArray.msg"
  "msg/PoseXYZRPY.msg"
  "msg/Command.msg"
  "msg/JointCommand.msg"
  "msg/JointCommandArray.msg"
)

set(srv_files
)

set(action_files
  "action/Command.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES
  builtin_interfaces
  std_msgs
  sensor_msgs
  geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
