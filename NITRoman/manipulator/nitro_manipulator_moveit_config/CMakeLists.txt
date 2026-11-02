cmake_minimum_required(VERSION 3.22)
project(nitro_manipulator_moveit_config)
find_package(ament_cmake REQUIRED)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
find_package(rclcpp REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(arm_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(robot_state_publisher REQUIRED)
# Add your executable with a unique name
add_executable(nitro_moveit_servo_command_generator src/moveit_servo_command_generator.cpp)

# Specify the dependencies
ament_target_dependencies(nitro_moveit_servo_command_generator rclcpp moveit_servo)

# Install the executable
install(TARGETS nitro_moveit_servo_command_generator
  DESTINATION lib/${PROJECT_NAME})
  # Add your executable for the IK solver
  add_executable(nitro_ik_solver src/ik_solver.cpp)
  ament_target_dependencies(nitro_ik_solver rclcpp moveit_servo arm_msgs)
  install(TARGETS nitro_ik_solver
    DESTINATION lib/${PROJECT_NAME})


    add_executable(set_from_ik src/set_from_ik.cpp)
    ament_target_dependencies(set_from_ik 
    rclcpp
    geometry_msgs
    moveit_core
    moveit_ros_planning_interface
    moveit_ros_planning
    robot_state_publisher
    arm_msgs
    tf2_ros)
    install(TARGETS set_from_ik
      DESTINATION lib/${PROJECT_NAME})

      
# Install launch files and config
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
