cmake_minimum_required(VERSION 3.5)
project(edie_behavior)

# Default to C99
# if(NOT CMAKE_C_STANDARD)
#   set(CMAKE_C_STANDARD 99)
# endif()

# Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unknown-pragmas)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(aeirobot_toolbox REQUIRED)
find_package(aeirobot_math REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(edie_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(BT_SOURCES
  src/action_nodes/remote_control_play.cpp
  src/action_nodes/move_to.cpp
  src/action_nodes/explore.cpp
  src/action_nodes/try_charge.cpp
  src/action_nodes/try_interact.cpp
  src/action_nodes/send_emergency_signal.cpp
  src/action_nodes/set_home_init_done.cpp
  src/action_nodes/try_navigation.cpp
  src/action_nodes/send_scan_command.cpp
  src/action_nodes/find_closest_marker.cpp
  src/action_nodes/docking.cpp
  src/action_nodes/set_target_marker_id.cpp
  src/action_nodes/collect_aruco_pose_data.cpp
  src/action_nodes/switch_vora_mode.cpp
  src/action_nodes/wait_action.cpp
  src/action_nodes/stare_at_target.cpp
  src/action_nodes/set_random_walk_flag.cpp
  src/action_nodes/process_fsr_touch.cpp
  src/action_nodes/play_emotion.cpp
  src/action_nodes/reset_waiting_status.cpp
  src/action_nodes/move_away_from_target.cpp
  src/action_nodes/set_wait_duration.cpp
  src/action_nodes/reset_angry_runaway.cpp

  src/condition_nodes/is_infront_with_target.cpp
  src/condition_nodes/is_vora_mode.cpp
  src/condition_nodes/is_face_detection_active.cpp
  src/condition_nodes/is_ready_to_action.cpp
  src/condition_nodes/is_motor_ready.cpp
  src/condition_nodes/is_robot_mode.cpp
  src/condition_nodes/is_emergency_detected.cpp
  src/condition_nodes/is_battery_charged_at_least.cpp
  src/condition_nodes/is_interaction_target_detected.cpp
  src/condition_nodes/is_navigation_done.cpp
  src/condition_nodes/is_pose_correction_done.cpp
  src/condition_nodes/is_aruco_visible.cpp
  src/condition_nodes/is_home_init_done.cpp
  src/condition_nodes/is_real_middle.cpp
  src/condition_nodes/is_scan_result_received.cpp
  src/condition_nodes/is_robot_in_area.cpp
  src/condition_nodes/is_charging.cpp
  src/condition_nodes/is_stt_listening.cpp
  src/condition_nodes/is_llm_state.cpp
  src/condition_nodes/is_random_walk_needed.cpp
  src/condition_nodes/is_human_visible.cpp
  src/condition_nodes/is_waiting_for_touch.cpp
  src/condition_nodes/is_in_interaction_mode.cpp
  src/condition_nodes/is_angry_runaway.cpp
  src/condition_nodes/is_station_out_value.cpp
  
  src/edie.cpp
  src/register_nodes.cpp
)

add_executable(edie_behavior_node src/edie_behavior_node.cpp ${BT_SOURCES})

target_include_directories(edie_behavior_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(edie_behavior_node
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "diagnostic_msgs"
  "edie_msgs"
  "ament_index_cpp"
  "behaviortree_cpp"
  "aeirobot_toolbox"
  "aeirobot_math"
)

install(TARGETS edie_behavior_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
  config
  launch
  DESTINATION share/${PROJECT_NAME}
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
