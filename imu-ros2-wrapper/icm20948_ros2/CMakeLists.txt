cmake_minimum_required(VERSION 3.8)
project(icm20948_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(aeirobot_toolbox REQUIRED)
find_package(aeirobot_math REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rcl_interfaces REQUIRED)
# Python embedding
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

# include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${Python3_INCLUDE_DIRS}
)

# find the pkgconfig and use this tool to find the third party packages
find_package(PkgConfig REQUIRED)
pkg_search_module(GPIOD REQUIRED libgpiod)

# include all library header directories
set(LIB_INC_DIRS
    ${GPIOD_INCLUDE_DIRS}
)

# include all linked libraries
set(LIBS
    ${GPIOD_LIBRARIES}
)

# make 'imu_utils' as static library
add_library(imu_utils STATIC ../imu_utils.cpp)
target_include_directories(imu_utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
ament_target_dependencies(imu_utils aeirobot_toolbox aeirobot_math)

# add the executable
add_executable(icm20948_read_node src/icm20948_imu_reader.cpp  ${MAIN})
ament_target_dependencies(icm20948_read_node rclcpp sensor_msgs aeirobot_toolbox aeirobot_math)

# imu_utils 라이브러리 링크
target_link_libraries(icm20948_read_node
  yaml-cpp
  aeirobot_toolbox::aeirobot_toolbox
  imu_utils
  ${Python3_LIBRARIES}
)

target_include_directories(icm20948_read_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../..>
)

# install targets
install(TARGETS
  icm20948_read_node
  DESTINATION lib/${PROJECT_NAME}
)

# install resource directories
install(
  DIRECTORY
    config
    launch
  DESTINATION
    share/${PROJECT_NAME}/
)

# install Python driver modules
install(
  DIRECTORY python_modules/
  DESTINATION share/${PROJECT_NAME}/python_modules
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
