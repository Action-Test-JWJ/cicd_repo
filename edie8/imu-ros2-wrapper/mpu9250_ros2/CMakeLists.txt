cmake_minimum_required(VERSION 3.8)
project(mpu9250_ros2)

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
find_package(nav_msgs REQUIRED)

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

# include all header directories
set(INC_DIRS
    ${LIB_INC_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/mpu9250/src
    ${CMAKE_CURRENT_SOURCE_DIR}/mpu9250/interface
    ${CMAKE_CURRENT_SOURCE_DIR}/mpu9250/example
    ${CMAKE_CURRENT_SOURCE_DIR}/mpu9250/test
    ${CMAKE_CURRENT_SOURCE_DIR}/mpu9250/project/raspberrypi4b/interface/inc
)

include_directories(
  include
  ${INC_DIRS}
)

# include all sources files
file(GLOB SRCS
     ${CMAKE_CURRENT_SOURCE_DIR}/mpu9250/src/*.c
     ${CMAKE_CURRENT_SOURCE_DIR}/mpu9250/project/raspberrypi4b/driver/src/raspberrypi4b_driver_mpu9250_interface.c
     ${CMAKE_CURRENT_SOURCE_DIR}/mpu9250/project/raspberrypi4b/interface/src/iic.c
)

# include executable source
file(GLOB MAIN
     ${SRCS}
     ${CMAKE_CURRENT_SOURCE_DIR}/mpu9250/example/*.c
     ${CMAKE_CURRENT_SOURCE_DIR}/mpu9250/test/*.c
     ${CMAKE_CURRENT_SOURCE_DIR}/mpu9250/interface/src/*.c
     ${CMAKE_CURRENT_SOURCE_DIR}/mpu9250/driver/src/*.c
    # ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp  // Add this line
    #  ${CMAKE_CURRENT_SOURCE_DIR}/mpu9250/project/raspberrypi4b/src/main.c
)

# make as library
add_library(imu_utils STATIC ../imu_utils.cpp)
target_include_directories(imu_utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
ament_target_dependencies(imu_utils aeirobot_toolbox aeirobot_math)

# add the executable
add_executable(imu_read_node src/imu_reader.cpp  ${MAIN})
ament_target_dependencies(imu_read_node rclcpp sensor_msgs aeirobot_toolbox aeirobot_math)

add_executable(imu_offset_init_node src/imu_offset_init.cpp)
ament_target_dependencies(imu_offset_init_node rclcpp std_msgs rcl_interfaces)

# imu_utils 라이브러리 링크
target_link_libraries(imu_read_node imu_utils)

target_include_directories(imu_read_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../..>
)

# install targets
install(TARGETS
  imu_read_node
  imu_offset_init_node
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(imu_read_node yaml-cpp aeirobot_toolbox::aeirobot_toolbox aeirobot_math::aeirobot_math)

install(
  DIRECTORY
    config
    launch
  DESTINATION
    share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
