cmake_minimum_required(VERSION 3.5)
project(nitro_description)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(flipper_msgs REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY xacro
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY openarm_meshes
  DESTINATION share/${PROJECT_NAME}/
)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/target_joint_state_publisher.cpp")
  add_executable(target_joint_state_publisher src/target_joint_state_publisher.cpp)
  ament_target_dependencies(target_joint_state_publisher rclcpp std_msgs sensor_msgs flipper_msgs)

  install(TARGETS target_joint_state_publisher
    DESTINATION lib/${PROJECT_NAME}
  )
endif()
ament_package()