cmake_minimum_required(VERSION 3.8)
project(edie_imu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(aeirobot_toolbox REQUIRED)
find_package(aeirobot_math REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED) 
find_package(rcl_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

# find the pkgconfig and use this tool to find the third party packages
find_package(PkgConfig REQUIRED)
pkg_search_module(GPIOD REQUIRED libgpiod)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# add the executable
add_executable(edie_imu_node src/edie_imu_node.cpp src/kalman_filter.cpp src/low_pass_filter.cpp src/one_euro_filter.cpp)
ament_target_dependencies(edie_imu_node rclcpp sensor_msgs aeirobot_toolbox nav_msgs Eigen3 tf2_ros tf2_geometry_msgs aeirobot_math)

add_executable(edie_imu_odometry_node src/edie_imu_odometry_node.cpp)
ament_target_dependencies(edie_imu_odometry_node rclcpp sensor_msgs aeirobot_toolbox nav_msgs Eigen3 tf2_ros tf2_geometry_msgs aeirobot_math)

add_executable(imu_jitter_monitor src/imu_jitter_monitor.cpp)
ament_target_dependencies(imu_jitter_monitor rclcpp sensor_msgs cv_bridge OpenCV)

# install targets
install(TARGETS
  edie_imu_node
  edie_imu_odometry_node
  imu_jitter_monitor
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(edie_imu_node yaml-cpp aeirobot_toolbox::aeirobot_toolbox)
target_link_libraries(edie_imu_odometry_node yaml-cpp aeirobot_toolbox::aeirobot_toolbox)
target_link_libraries(imu_jitter_monitor ${OpenCV_LIBS})

install(
  DIRECTORY
    rviz
    config
    launch
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_package()