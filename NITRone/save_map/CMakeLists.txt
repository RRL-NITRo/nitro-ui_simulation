cmake_minimum_required(VERSION 3.10)
project(save_map)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ps5_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

add_executable(save_map src/save_map.cpp)
ament_target_dependencies(save_map
  rclcpp
  sensor_msgs
  ps5_msgs
  pcl_conversions
)

# PCLのインクルードとリンク
target_include_directories(save_map PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(save_map ${PCL_LIBRARIES})

install(TARGETS save_map
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
