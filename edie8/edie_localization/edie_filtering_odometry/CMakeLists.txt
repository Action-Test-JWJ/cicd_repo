cmake_minimum_required(VERSION 3.8)
project(edie_filtering_odometry)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(aeirobot_toolbox REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(edie_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

# 컴파일러에게 특정 헤더 파일이 위치한 디렉토리를 알려줌
include_directories(include)

# 실행 파일 생성
add_executable(edie_filtering_odom_node src/edie_filtering_odom_node.cpp src/gaussian_modeling_filter.cpp src/odom_utils.cpp)
target_link_libraries(edie_filtering_odom_node yaml-cpp aeirobot_toolbox::aeirobot_toolbox tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs)

add_executable(edie_republishing_odom_node src/edie_republishing_odom_node.cpp src/odom_utils.cpp)
target_link_libraries(edie_republishing_odom_node aeirobot_toolbox::aeirobot_toolbox tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs)

add_executable(edie_path_tracking_node src/edie_path_tracking_node.cpp)
target_link_libraries(edie_path_tracking_node yaml-cpp aeirobot_toolbox::aeirobot_toolbox tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs)

add_executable(edie_correcting_odom_node src/edie_correcting_odom_node.cpp src/odom_utils.cpp)
target_link_libraries(edie_correcting_odom_node aeirobot_toolbox::aeirobot_toolbox tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs yaml-cpp)


# include 디렉토리 지정: 프로젝트 루트의 include 폴더를 추가
target_include_directories(edie_filtering_odom_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(edie_republishing_odom_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(edie_path_tracking_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(edie_correcting_odom_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(edie_filtering_odom_node 
  rclcpp 
  nav_msgs 
  geometry_msgs 
  tf2 
  yaml-cpp 
  aeirobot_toolbox
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(edie_republishing_odom_node 
  rclcpp 
  nav_msgs 
  geometry_msgs 
  tf2 
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(edie_path_tracking_node 
  rclcpp 
  nav_msgs 
  geometry_msgs 
  tf2 
  yaml-cpp 
  aeirobot_toolbox
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(edie_correcting_odom_node 
  rclcpp 
  nav_msgs 
  geometry_msgs 
  tf2 
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  edie_msgs
  ament_index_cpp
  yaml-cpp
)

# 설치 후 다른 패키지가 헤더 파일을 사용할 수 있도록 복사
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
  install(
    DIRECTORY include
    DESTINATION include
  )
else()
  message(WARNING "Include directory does not exist: ${CMAKE_CURRENT_SOURCE_DIR}/include")
endif()

# install targets
install(TARGETS
  edie_filtering_odom_node
  edie_republishing_odom_node
  edie_path_tracking_node
  edie_correcting_odom_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

