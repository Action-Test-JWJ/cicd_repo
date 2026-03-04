cmake_minimum_required(VERSION 3.5)
project(edie_sound)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(edie_msgs REQUIRED)

# JSONCPP
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
link_libraries(${JSONCPP_LIBRARIES})

include_directories(
  include
  ${JSONCPP_INCLUDE_DIRS}
)

add_executable(edie_sound_node src/edie_sound_node.cpp src/sound.cpp)
ament_target_dependencies(edie_sound_node rclcpp std_msgs edie_msgs)
target_link_libraries(edie_sound_node ${JSONCPP_LIBRARIES} ament_index_cpp::ament_index_cpp)

install(DIRECTORY 
  config
  sounds
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  edie_sound_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()