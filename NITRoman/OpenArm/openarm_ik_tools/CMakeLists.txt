cmake_minimum_required(VERSION 3.16)
project(openarm_ik_tools)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(arm_msgs REQUIRED)
find_package(ps5_msgs REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(flipper_msgs REQUIRED)


# 実行ファイル
add_executable(openarm_set_from_ikleft src/openarm_set_from_ikleft.cpp)
ament_target_dependencies(openarm_set_from_ikleft
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  moveit_core
  moveit_ros_planning
  arm_msgs
)

add_executable(openarm_set_from_ikright src/openarm_set_from_ikright.cpp)
ament_target_dependencies(openarm_set_from_ikright
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  moveit_core
  moveit_ros_planning
  arm_msgs
)

add_executable(body_set_from_ik src/body_set_from_ik.cpp)
ament_target_dependencies(body_set_from_ik
  rclcpp
  geometry_msgs
  sensor_msgs
  Eigen3
  arm_msgs
)


add_executable(openarm_joy2commandright src/openarm_joy2commandright.cpp)
ament_target_dependencies(openarm_joy2commandright
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  moveit_core
  moveit_ros_planning
  arm_msgs
  ps5_msgs
)

add_executable(openarm_joy2commandleft src/openarm_joy2commandleft.cpp)
ament_target_dependencies(openarm_joy2commandleft
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  moveit_core
  moveit_ros_planning
  arm_msgs
  ps5_msgs
)

add_executable(joy2commandflipper src/joy2commandflipper.cpp)
ament_target_dependencies(joy2commandflipper
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  moveit_core
  moveit_ros_planning
  arm_msgs
  ps5_msgs
  flipper_msgs
)




# インストール設定
install(TARGETS openarm_set_from_ikleft openarm_set_from_ikright openarm_joy2commandright openarm_joy2commandleft body_set_from_ik joy2commandflipper
  DESTINATION lib/${PROJECT_NAME}
)
# launch ディレクトリを正しくコピー
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)


# ament パッケージ宣言
ament_package()
