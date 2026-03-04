cmake_minimum_required(VERSION 3.8)
project(edie_localization_manager)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(edie_msgs REQUIRED)
find_package(aeirobot_toolbox REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(edie8_parameters REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

set(ROS2_WS_ROOT $ENV{ROS_WS})
add_definitions(-DROS2_WS_ROOT="${ROS2_WS_ROOT}")

set(dependencies
    rclcpp
    std_msgs
    tf2_msgs
    geometry_msgs
    visualization_msgs
    nav_msgs
    aeirobot_toolbox
    diagnostic_msgs
    edie_msgs
)

# Static library
add_library(${PROJECT_NAME}_static_lib STATIC
    src/edie_localization_manager.cpp
    src/core/pose_evaluation.cpp
    src/core/pose_fusion.cpp
    src/core/pose_set.cpp
    src/core/pose_prediction.cpp
    src/utility/parameter_loader.cpp
    src/utility/info_parser.cpp
    src/utility/odom_tools.cpp
    src/utility/tools.cpp
)

target_include_directories(${PROJECT_NAME}_static_lib
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Executable
add_executable(${PROJECT_NAME}_node src/edie_localization_manager_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}_static_lib
)

# Dependencies
ament_target_dependencies(${PROJECT_NAME}_static_lib ${dependencies})
ament_target_dependencies(${PROJECT_NAME}_node ${dependencies})

# Install
install(TARGETS ${PROJECT_NAME}_static_lib
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    INCLUDES DESTINATION include
)

install(TARGETS
    ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

# Export
ament_export_libraries(${PROJECT_NAME}_static_lib)
ament_export_targets(export_${PROJECT_NAME})

# Package
ament_package()
