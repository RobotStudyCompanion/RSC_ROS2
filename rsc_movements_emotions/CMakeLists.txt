cmake_minimum_required(VERSION 3.8)
project(rsc_movements_emotions)

# Add compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)  # Python-based ROS 2 dependency

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip linter checks (remove these lines when adding license/copyright)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


install(
  DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()

