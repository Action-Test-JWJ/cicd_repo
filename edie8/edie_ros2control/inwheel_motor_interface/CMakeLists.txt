cmake_minimum_required(VERSION 3.5)
project(inwheel_motor_hardware)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(transmission_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(aeirobot_toolbox REQUIRED)
find_package(aeirobot_math REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/inwheel_motor_hardware.cpp
  src/inwheel_motor_can.cpp
  src/epoll_event_loop.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
  ${YAML_CPP_INCLUDE_DIRS}  # Include directories for yaml-cpp
)
ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  rclcpp_action
  hardware_interface
  transmission_interface
  pluginlib
  yaml-cpp
  # alice4_tuning_module_msgs
  aeirobot_toolbox
  aeirobot_math
)
target_link_libraries(
  ${PROJECT_NAME}
  ${YAML_CPP_LIBRARIES}  # Link yaml-cpp libraries
)

pluginlib_export_plugin_description_file(hardware_interface inwheel_motor_hardware.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
  ${YAML_CPP_LIBRARIES}  # Export yaml-cpp libraries
)
ament_export_dependencies(
  rclcpp
  hardware_interface
  aeirobot_toolbox
  aeirobot_math
  pluginlib
  yaml-cpp  # Export yaml-cpp dependency
)

ament_package()
