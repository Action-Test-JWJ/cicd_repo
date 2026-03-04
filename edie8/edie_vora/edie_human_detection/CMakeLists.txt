cmake_minimum_required(VERSION 3.8)
project(edie_human_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)  
find_package(geometry_msgs REQUIRED)                                                                           
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(aeirobot_toolbox REQUIRED)

# Install Python executables
install(
  DIRECTORY src/gtilib                  # Include the gtilib folder
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)
install(
  DIRECTORY src/gyurinet                # Include the gyurinet folder
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

# Install Python executables
install(
  PROGRAMS 
    src/gyurinet/gtilib/reader.py
    src/gyurinet/gtilib/util.py
    src/box.py
    src/demo.py
    src/detector.py
    src/box_util.py
  DESTINATION lib/${PROJECT_NAME}
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
              GROUP_EXECUTE GROUP_READ
              WORLD_EXECUTE WORLD_READ
)

install(
  PROGRAMS 
    src/edie_human_detection_node.py
  RENAME 
    human_detection_node
  DESTINATION lib/${PROJECT_NAME}
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
              GROUP_EXECUTE GROUP_READ
              WORLD_EXECUTE WORLD_READ
)
  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()