cmake_minimum_required(VERSION 3.5)
project(edie_display)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-std=c++14)
endif()

# 필요한 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(edie_msgs REQUIRED)
# find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
# find_package(OpenGL REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
pkg_check_modules(SDL2 REQUIRED sdl2)
pkg_check_modules(SDL2IMAGE REQUIRED SDL2_image)

# 포함 디렉토리 설정
include_directories(
  include
  ${edie_msgs_INCLUDE_DIRS}
  # ${OpenCV_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
  ${SDL2_INCLUDE_DIRS}
  ${SDL2IMAGE_INCLUDE_DIRS}
)

set(PROJECT_LIB_FILES
  src/emotion.cpp
  src/parser.cpp
)

# 라이브러리 추가
add_library(${PROJECT_NAME}_lib
  ${PROJECT_LIB_FILES}
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  std_msgs
  edie_msgs
)

target_link_libraries(${PROJECT_NAME}_lib
  # ${OpenCV_LIBRARIES}
  ${SDL2_LIBRARIES}
  ${SDL2IMAGE_LIBRARIES}
  ${JSONCPP_LIBRARIES}
)

# 실행 파일 추가
add_executable(edie_display_node src/edie_display_node.cpp)
ament_target_dependencies(edie_display_node rclcpp std_msgs edie_msgs)
target_link_libraries(edie_display_node 
  ${PROJECT_NAME}_lib
  ament_index_cpp::ament_index_cpp
  Boost::filesystem
)

# 설치 디렉토리 설정
install(DIRECTORY
  images
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  edie_display_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
