################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(aeirobot_math)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if($ENV{ROS_DISTRO} STREQUAL "foxy")
  add_definitions(-DROS_DISTRO=foxy)
elseif($ENV{ROS_DISTRO} STREQUAL "humble")
  add_definitions(-DROS_DISTRO=humble)
endif()

set(ROBOT_TYPE "edie" CACHE STRING "Select robot type: edie or alice")
message(STATUS "Building for ROBOT_TYPE: ${ROBOT_TYPE}")

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(aeirobot_toolbox REQUIRED)
pkg_check_modules(yaml_cpp yaml-cpp REQUIRED)

set(dependencies
  rclcpp
  Eigen3
  aeirobot_toolbox
  osqp_vendor
)

if(ROBOT_TYPE STREQUAL "alice")
  find_package(osqp_vendor REQUIRED)
endif()

################################################################################
# Build
################################################################################
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${yaml_cpp_INCLUDE_DIRS}
)
if(ROBOT_TYPE STREQUAL "alice")
  include_directories(${osqp_vendor_INCLUDE_DIRS})
endif()

add_library(${PROJECT_NAME} SHARED
  src/algebra/linear_algebra.cpp
  src/algebra/lie_algebra.cpp
  src/trajectory/bezier_curve.cpp
  src/algebra/incremental_jacobian.cpp
  src/trajectory/fifth_order_polynomial_trajectory.cpp
  src/trajectory/ninth_order_polynomial_trajectory.cpp
  src/trajectory/seventh_order_polynomial_trajectory.cpp
  src/trajectory/minimum_jerk_trajectory_with_via_point.cpp
  src/trajectory/minimum_jerk_trajectory.cpp
  src/trajectory/simple_trapezoidal_velocity_profile.cpp
  src/trajectory/trajectory_calculator.cpp
  src/trajectory/fifth_order_trajectory_generate.cpp
  # src/optimizer/riccati_solver.cpp
  # src/optimizer/qpwithosqp_solver.cpp
)
if(ROBOT_TYPE STREQUAL "alice")
  target_sources(${PROJECT_NAME} PRIVATE
    src/optimizer/qpwithosqp_solver.cpp
  )
endif()

if(ROBOT_TYPE STREQUAL "edie")
  target_sources(${PROJECT_NAME} PRIVATE
    src/trajectory/fifth_order_trajectory_generate.cpp
  )
endif()

target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC
    rclcpp::rclcpp
    aeirobot_toolbox::aeirobot_toolbox
    ${Eigen3_LIBRARIES}
) #시스템 라이브러리

if(ROBOT_TYPE STREQUAL "alice")
  target_link_libraries(${PROJECT_NAME} PRIVATE osqp::osqp)
endif()
# ament_target_dependencies(${PROJECT_NAME} ${dependencies}) # ROS2 라이브러리

################################################################################
# Install
################################################################################

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> # 소스 빌드
  $<INSTALL_INTERFACE:include>) # 설치 후

install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()