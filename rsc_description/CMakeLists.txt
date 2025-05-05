cmake_minimum_required(VERSION 3.8)
project(rsc_description)

# Add compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)

find_package(rclpy REQUIRED)  # Python-based ROS 2 dependency
find_package(sensor_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OGRE REQUIRED)




include_directories(include)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip linter checks (remove these lines when adding license/copyright)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install URDF, meshes, and RViz configuration files
install(
  DIRECTORY urdf meshes rviz launch textures src
  DESTINATION share/${PROJECT_NAME}
)

# Install package.xml
install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)




# Install Python scripts


# Mark this as an ament package
ament_package()

