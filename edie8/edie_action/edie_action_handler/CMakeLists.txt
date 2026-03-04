cmake_minimum_required(VERSION 3.5)
project(edie_action_handler)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(edie_msgs REQUIRED)
find_package(aeirobot_toolbox REQUIRED)
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(
  include
)

# Add executable
add_executable(edie_action_handler_node 
  src/main.cpp
  src/edie_action_handler_callback.cpp
  src/edie_action_handler_node.cpp
  src/action_yaml_parser.cpp
)
ament_target_dependencies(edie_action_handler_node rclcpp std_msgs edie_msgs aeirobot_toolbox)
target_link_libraries(edie_action_handler_node yaml-cpp)

# Install
install(TARGETS
edie_action_handler_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    data
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_package()
