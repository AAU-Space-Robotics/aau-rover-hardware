cmake_minimum_required(VERSION 3.8)
project(hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# Add the executable for the C++ node
add_executable(placeholder_node src/placeholder_node.cpp)
ament_target_dependencies(placeholder_node rclcpp std_msgs)

# Install the C++ node executable
install(TARGETS
  placeholder_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install python nodes
install(PROGRAMS
  scripts/pose_transform.py
  scripts/motors.py
  scripts/ackermann.py
  scripts/joy_to_cmd_vel.py
  DESTINATION lib/${PROJECT_NAME})

# Install Python launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
