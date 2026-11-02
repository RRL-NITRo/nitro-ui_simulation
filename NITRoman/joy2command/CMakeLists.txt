cmake_minimum_required(VERSION 3.8)
project(joy2command)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# Set C++ standard

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ps5_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(arm_msgs REQUIRED)
find_package(flipper_msgs REQUIRED)

include_directories(
  include
)

# add_executable(ps5_joy_publisher src/ps5_joy_publisher.cpp)
add_executable(joy2command src/joy2command.cpp)
ament_target_dependencies(joy2command 
rclcpp std_msgs geometry_msgs sensor_msgs arm_msgs yaml-cpp ps5_msgs flipper_msgs)

add_executable(joy2commandnitromanbody src/joy2commandnitromanbody.cpp)
ament_target_dependencies(joy2commandnitromanbody
rclcpp std_msgs geometry_msgs sensor_msgs yaml-cpp ps5_msgs )

add_executable(joy2command_drone src/joy2command_drone.cpp)
ament_target_dependencies(joy2command_drone
rclcpp std_msgs geometry_msgs sensor_msgs arm_msgs yaml-cpp ps5_msgs flipper_msgs)

add_executable(PanTilt_command_publisher src/PanTilt_command_publisher.cpp)
ament_target_dependencies(PanTilt_command_publisher
rclcpp std_msgs ps5_msgs)

install(TARGETS
  joy2command
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  joy2commandnitromanbody
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  joy2command_drone
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  PanTilt_command_publisher
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# ament_target_dependencies(ps5_joy_publisher rclcpp std_msgs ps5_msgs sensor_msgs yaml-cpp geometry_msgs)
# target_link_libraries(ps5_joy_publisher yaml-cpp)
# install(TARGETS
#   ps5_joy_publisher
#   DESTINATION lib/${PROJECT_NAME})


#   install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}/
# )
# install(DIRECTORY config
#   DESTINATION share/${PROJECT_NAME}/
# )

ament_package()
