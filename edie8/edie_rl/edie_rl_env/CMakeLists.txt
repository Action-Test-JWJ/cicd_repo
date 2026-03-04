cmake_minimum_required(VERSION 3.5)
project(edie_rl_env)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

include_directories(include
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(edie_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_library(YAML_CPP_LIBRARY
  NAMES YAML_CPP
  PATHS ${YAML_CPP_LIBRARY_DIRS}
)
link_directories(${YAML_CPP_LIBRARY_DIRS})

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

include_directories(
  include
  ${YAML_CPP_INCLUDEDIR}
)

#Build
add_executable(edie_reinforcement_test_env_node src/edie_reinforcement_test_env_node.cpp)
ament_target_dependencies(edie_reinforcement_test_env_node rclcpp std_msgs std_srvs edie_msgs)
target_link_libraries(edie_reinforcement_test_env_node ${YAML_CPP_LIBRARIES})

add_executable(edie_reinforcement_env_node src/edie_reinforcement_env_node.cpp)
ament_target_dependencies(edie_reinforcement_env_node rclcpp std_msgs std_srvs edie_msgs)
target_link_libraries(edie_reinforcement_env_node ${YAML_CPP_LIBRARIES})

#add_executable(edie_rl_virtual_env_node src/edie_rl_virtual_env_node.cpp)
#ament_target_dependencies(edie_rl_virtual_env_node rclcpp std_msgs std_srvs edie_msgs)
#target_link_libraries(edie_rl_virtual_env_node ${YAML_CPP_LIBRARIES})

add_executable(edie_rl_bt_node src/edie_rl_bt_node.cpp)
ament_target_dependencies(edie_rl_bt_node rclcpp std_msgs std_srvs edie_msgs)
target_link_libraries(edie_rl_bt_node ${YAML_CPP_LIBRARIES})

install(TARGETS
  edie_reinforcement_test_env_node
  edie_reinforcement_env_node
  #edie_rl_virtual_env_node
  edie_rl_bt_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

add_definitions(-DSOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
