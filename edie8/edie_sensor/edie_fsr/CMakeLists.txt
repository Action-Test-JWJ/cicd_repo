cmake_minimum_required(VERSION 3.8)
project(edie_fsr)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(aeirobot_toolbox REQUIRED)

include_directories(include)

add_executable(edie_fsr_sensor_node src/main.cpp src/edie_fsr_sensor.cpp)
ament_target_dependencies(edie_fsr_sensor_node rclcpp std_msgs aeirobot_toolbox)
target_link_libraries(edie_fsr_sensor_node)

install(TARGETS
  edie_fsr_sensor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_package()