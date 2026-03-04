cmake_minimum_required(VERSION 3.16)
project(edie_hardware LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  aeirobot_toolbox
  edie_msgs
  std_msgs
  std_srvs
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  edie_hardware
  SHARED
  src/edie_hardware.cpp
  src/serial_comms.cpp
  src/protocol_commands.cpp
  src/protocol_status.cpp
  src/protocol_callbacks.cpp
  src/debug_callback.cpp
)
target_compile_features(edie_hardware PUBLIC cxx_std_17)
target_include_directories(edie_hardware PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/edie_hardware>
)
ament_target_dependencies(
  edie_hardware PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(edie_hardware
  PUBLIC
    serial
  PRIVATE
    Threads::Threads
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "EDIE_HARDWARE_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface edie_hardware.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/edie_hardware
)

install(TARGETS edie_hardware
  EXPORT export_edie_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_edie_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()