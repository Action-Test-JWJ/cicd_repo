cmake_minimum_required(VERSION 3.8)
project(edie_laser)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(aeirobot_toolbox REQUIRED)

include_directories(
  include
  vl53l0x-linux/include
  vl53l1x-linux/include
  vl6180x-linux/include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Add the vl53l0x-linux library
add_library(vl53l0x-linux STATIC
  vl53l0x-linux/src/I2Cdev.cpp
  vl53l0x-linux/src/VL53L0X.cpp
)

# Define vl53l1x_linux library
add_library(vl53l1x-linux STATIC
  vl53l1x-linux/src/vl53l1x.cpp
)

# Define vl6180x_linux library
add_library(vl6180x_linux STATIC
  vl6180x-linux/src/vl6180.cpp
)

set_target_properties(vl53l0x-linux PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO
)

# Include directories for vl53l0x-linux library
target_include_directories(vl53l0x-linux PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vl53l0x-linux>
  $<INSTALL_INTERFACE:vl53l0x-linux>
)

###################
##### VL53L0X #####
###################
add_executable(edie_laser_node_vl53l0x
  src/main_vl53l0x.cpp
  src/edie_laser_node_vl53l0x.cpp
  src/LaserSensor.cpp
  src/kalman_filter.cpp
)

target_include_directories(edie_laser_node_vl53l0x PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vl53l0x-linux>
)

target_link_libraries(edie_laser_node_vl53l0x vl53l0x-linux)
ament_target_dependencies(edie_laser_node_vl53l0x rclcpp std_msgs aeirobot_toolbox)

###################
##### VL53L1X #####
###################
add_executable(edie_laser_node_vl53l1x
  src/main_vl53l1x.cpp
  src/edie_laser_node_vl53l1x.cpp
  src/kalman_filter.cpp
)

target_include_directories(edie_laser_node_vl53l1x PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vl53l1x-linux>
)

target_link_libraries(edie_laser_node_vl53l1x vl53l1x-linux)
ament_target_dependencies(edie_laser_node_vl53l1x rclcpp std_msgs sensor_msgs aeirobot_toolbox)

###################
##### VL6180X #####
###################
add_executable(edie_laser_node_vl6180x
  src/main_vl6180x.cpp
  src/edie_laser_node_vl6180x.cpp
  src/kalman_filter.cpp
)

# Link the vl6180x_linux library to the executable
target_include_directories(edie_laser_node_vl6180x PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vl6180x-linux/include>
)

target_link_libraries(edie_laser_node_vl6180x vl6180x_linux)
ament_target_dependencies(edie_laser_node_vl6180x rclcpp std_msgs aeirobot_toolbox)

# Install targets
install(TARGETS
  edie_laser_node_vl53l0x
  edie_laser_node_vl53l1x
  edie_laser_node_vl6180x
  vl53l0x-linux
  vl53l1x-linux
  vl6180x_linux
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
