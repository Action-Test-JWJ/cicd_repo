cmake_minimum_required(VERSION 3.8)
project(edie_mobile)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(aeirobot_toolbox REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${YAML_CPP_INCLUDE_DIRS}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(edie_mobile_node src/main.cpp src/edie_mobile_callback.cpp src/edie_mobile_math.cpp src/edie_pathplan.cpp src/edie_mobile.cpp)
ament_target_dependencies(edie_mobile_node rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs tf2 tf2_ros aeirobot_toolbox)
target_link_libraries(edie_mobile_node yaml-cpp)

install(TARGETS
  edie_mobile_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY 
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
