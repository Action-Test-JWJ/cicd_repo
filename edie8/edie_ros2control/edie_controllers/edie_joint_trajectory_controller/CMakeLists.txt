cmake_minimum_required(VERSION 3.8)
project(edie_joint_trajectory_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
set(THIS_PACKAGE_INCLUDE_DEPENDS
  angles
  control_msgs
  control_toolbox
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  rsl
  tl_expected
  trajectory_msgs
  urdf
  aeirobot_toolbox
  edie_msgs
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Generate parameter library
generate_parameter_library(edie_joint_trajectory_controller_parameters
  src/edie_joint_trajectory_controller_parameters.yaml
  include/edie_joint_trajectory_controller/validate_jtc_parameters.hpp
)

# Build main library
add_library(${PROJECT_NAME} SHARED
  src/joint_trajectory_controller.cpp
  src/trajectory/linear_trajectory_generate.cpp
  src/trajectory/sine_trajectory_generate.cpp
  src/trajectory/third_order_trajectory_generate.cpp
  src/trajectory/fifth_order_trajectory_generate.cpp
  src/trajectory/min_jerk_trajectory_generate.cpp
  src/trajectory/lspb_trajectory_generate.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/edie_joint_trajectory_controller>
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(${PROJECT_NAME} PUBLIC edie_joint_trajectory_controller_parameters)

# Install targets
install(TARGETS
  ${PROJECT_NAME}
  edie_joint_trajectory_controller_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Plugin description file
install(FILES controller_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export
pluginlib_export_plugin_description_file(controller_interface controller_plugins.xml)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

# Lint/test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
