cmake_minimum_required(VERSION 3.8)
project(rviz_video_screen)

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

add_library(video_screen_display SHARED
  src/rviz_video_screen/video_screen_display.cpp
)

target_compile_definitions(video_screen_display PRIVATE RVIZ_COMMON_BUILDING_LIBRARY)



ament_target_dependencies(video_screen_display
  rclcpp
  sensor_msgs
  rviz_common
  pluginlib
  OpenCV
  cv_bridge
)

target_include_directories(video_screen_display PRIVATE
  ${OGRE_INCLUDE_DIRS}
)

target_link_libraries(video_screen_display
  ${OGRE_LIBRARIES}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip linter checks (remove these lines when adding license/copyright)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install package.xml
install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)



pluginlib_export_plugin_description_file(rviz_common plugins.xml)

install(TARGETS video_screen_display
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

# Install Python scripts


# Mark this as an ament package
ament_package()

