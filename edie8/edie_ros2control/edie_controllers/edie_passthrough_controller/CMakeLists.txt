cmake_minimum_required(VERSION 3.16)
project(edie_passthrough_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  control_msgs
  generate_parameter_library
  controller_interface
  parameter_traits
  realtime_tools
  std_msgs
)

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Add library of the controller and export it
generate_parameter_library(edie_passthrough_controller_parameters
  src/passthrough_controller_parameters.yaml
)

add_library(edie_passthrough_controller SHARED
  src/passthrough_controller.cpp
)
target_include_directories(edie_passthrough_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(edie_passthrough_controller PUBLIC edie_passthrough_controller_parameters)
ament_target_dependencies(edie_passthrough_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(controller_interface passthrough_controller.xml)


# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/passthrough_controller
)

install(TARGETS
    edie_passthrough_controller
    edie_passthrough_controller_parameters
  EXPORT export_edie_passthrough_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(FILES
  passthrough_controller.xml
  DESTINATION share/${PROJECT_NAME}
)

## EXPORTS
ament_export_targets(export_edie_passthrough_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
