cmake_minimum_required(VERSION 3.8)
project(flipper_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(flipper_msgs REQUIRED)

add_executable(flipper_control src/flipper_control.cpp)
ament_target_dependencies(flipper_control rclcpp std_msgs sensor_msgs flipper_msgs)
install(TARGETS
  flipper_control
  DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)


add_executable(joint_state_merge src/joint_state_merge.cpp)
ament_target_dependencies(joint_state_merge rclcpp std_msgs sensor_msgs flipper_msgs)
install(TARGETS
  joint_state_merge
  DESTINATION lib/${PROJECT_NAME})

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
