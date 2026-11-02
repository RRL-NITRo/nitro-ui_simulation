cmake_minimum_required(VERSION 3.8)
project(joy_handler)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ps5_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(ps5_joy_publisher src/ps5_joy_publisher.cpp)


ament_target_dependencies(ps5_joy_publisher rclcpp std_msgs ps5_msgs sensor_msgs yaml-cpp geometry_msgs)
target_link_libraries(ps5_joy_publisher yaml-cpp)
install(TARGETS
  ps5_joy_publisher
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
