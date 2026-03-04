################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(aeirobot_robotics)

find_package(ament_cmake REQUIRED)
# Check robot type and skip build if necessary
set(ROBOT_TYPE "edie" CACHE STRING "Select robot type: edie or alice")
if(ROBOT_TYPE STREQUAL "edie")
  message(STATUS "Skipping package aeirobot_hand_msgs for ROBOT_TYPE=${ROBOT_TYPE}")
  return()
endif()

# “python310” 컴포넌트의 헤더 파일명을 명시적으로 지정.
# pinocchio가 Boost.Python을 찾으므로,
# Boost_PYTHON310_HEADER_NAME 설정이 위에 있어야 경고가 사라짐.
set(Boost_PYTHON310_HEADER_NAME "boost/python.hpp" CACHE STRING
    "Header file name for Boost.Python (python310)")

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -Wno-deprecated-enum-enum-conversion  # enum‐enum conversion 경고 비활성화
    -Wno-error=return-type                # non-void 함수 끝 경고를 에러로 승격하지 않음
  )
endif()

if($ENV{ROS_DISTRO} STREQUAL "foxy")
  add_definitions(-DROS_DISTRO=foxy)
elseif($ENV{ROS_DISTRO} STREQUAL "humble")
  add_definitions(-DROS_DISTRO=humble)
endif()

################################################################################
# Find and load build settings from external packages
###############################################################################
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(aeirobot_toolbox REQUIRED)
find_package(aeirobot_math REQUIRED)
find_package(pinocchio REQUIRED)
pkg_check_modules(yaml_cpp yaml-cpp REQUIRED)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${yaml_cpp_INCLUDE_DIRS}
  SYSTEM ${pinocchio_INCLUDE_DIRS}
)

set(dependencies
  rclcpp
  Eigen3
  aeirobot_toolbox
  aeirobot_math
  pinocchio
)

add_library(${PROJECT_NAME} SHARED
  src/kinematics/link_data.cpp
  src/kinematics/6dof_leg_kinematics.cpp
  src/kinematics/7dof_arm_kinematics.cpp
  src/kinematics/4dof_arm_kinematics.cpp
  src/linear_rotary_converter/linear_rotary_converter.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> # 패키지 자체 빌드
  $<INSTALL_INTERFACE:include> # 설치 후 다른 패키지
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  ${yaml_cpp_LIBRARIES}
  ${Eigen3_LIBRARIES}
  # osqp::osqp
) #시스템 라이브러리

ament_target_dependencies(${PROJECT_NAME} PUBLIC ${dependencies})
################################################################################
# Install
################################################################################
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/
  DESTINATION include
)

################################################################################
# Export
################################################################################
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

################################################################################
# Test
################################################################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

################################################################################
# Macro for ament package
################################################################################
ament_package()
