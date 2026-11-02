cmake_minimum_required(VERSION 3.10)
project(slam_stack)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS filters io)

add_executable(cloud_repacker src/cloud_repacker.cpp)
ament_target_dependencies(cloud_repacker
  rclcpp
  sensor_msgs
  pcl_conversions)

add_executable(pcd_writer src/pcd_writer.cpp)
ament_target_dependencies(pcd_writer
  rclcpp
  sensor_msgs
  pcl_conversions)

target_include_directories(cloud_repacker PRIVATE ${PCL_INCLUDE_DIRS})
target_include_directories(pcd_writer PRIVATE ${PCL_INCLUDE_DIRS})

target_link_libraries(cloud_repacker ${PCL_LIBRARIES})
target_link_libraries(pcd_writer ${PCL_LIBRARIES})

install(TARGETS cloud_repacker pcd_writer
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
