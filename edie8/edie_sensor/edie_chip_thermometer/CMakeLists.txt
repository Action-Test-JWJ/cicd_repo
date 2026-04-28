cmake_minimum_required(VERSION 3.8)
project(edie_chip_thermometer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcppp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(edie_chip_thermometer_node src/main.cpp src/edie_chip_thermometer_node.cpp)
ament_target_dependencies(edie_chip_thermometer_node rclcpp std_msgs)

install(TARGETS
  edie_chip_thermometer_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
