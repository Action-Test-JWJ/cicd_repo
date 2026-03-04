cmake_minimum_required(VERSION 3.8)
project(edie_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(edie_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(aeirobot_toolbox REQUIRED)
find_package(aeirobot_math REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${YAML_CPP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
) 
add_executable(edie_motion_handler_node src/main.cpp src/edie_motion_handler.cpp src/edie_motion_handler_action_callback.cpp src/edie_motion_handler_callback.cpp)
ament_target_dependencies(edie_motion_handler_node rclcpp rclcpp_action std_msgs edie_msgs sensor_msgs rclcpp_action aeirobot_toolbox aeirobot_math)
target_link_libraries(edie_motion_handler_node ${YAML_CPP_LIBRARIES} ament_index_cpp::ament_index_cpp)

install(
  DIRECTORY
    data
    launch
  DESTINATION
    share/${PROJECT_NAME}/
)

install(TARGETS
  edie_motion_handler_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()