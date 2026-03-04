cmake_minimum_required(VERSION 3.5)
project(edie_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(POLICY CMP0148) 
  cmake_policy(SET CMP0148 OLD)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Include directories
include_directories(include)

# Add library
add_library(${PROJECT_NAME}_lib src/edie_struct.cpp)
ament_target_dependencies(${PROJECT_NAME}_lib rclcpp std_msgs)

# Generate ROS2 messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EdieView.msg"
  "msg/EdieInfo.msg"
  "msg/ArucoPose.msg"
  "msg/ArucoPoseArray.msg"
  "msg/InferenceResult.msg"
  "msg/Yolov8Inference.msg"
  "msg/RunningNodes.msg"
  "msg/TerminatedNodes.msg"
  "msg/PoseEvaluation.msg"
  "msg/PoseWithInfoStamped.msg"
  "msg/BoolStamped.msg"
  "msg/HumanDetection.msg"
  "msg/HumanDetection2DArray.msg"
  "msg/EmotionDetection.msg"
  "msg/EmotionDetection2DArray.msg"
  "msg/FaceDetection.msg"
  "msg/FaceDetectionArray.msg"
  "msg/RobotLlmState.msg"
  "srv/CheckReady.srv"
  "srv/Reinforcement.srv"
  "srv/FilePath.srv"
  "srv/BtRl.srv"
  "srv/SetPidGain.srv"
  "srv/GetPidGain.srv"
  "srv/SwitchVoraMode.srv"
  "action/EdieMotionTrajectory.action"
  "action/EdieMotion.action"
  "action/HumanVoraInference.action"
  "action/EmotionVoraInference.action"
  DEPENDENCIES std_msgs geometry_msgs diagnostic_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# Install
install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

