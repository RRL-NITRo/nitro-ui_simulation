choreonoid_add_drone_controller(
    SampleDroneFlightController SampleDroneFlightController.cpp)

ament_target_dependencies(
    SampleDroneFlightController rclcpp geometry_msgs sensor_msgs)

choreonoid_add_simple_controller(
    ImuPublisherController ImuPublisherController.cpp MadgwickAHRS.cpp)


include_directories(${CHOREONOID_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/include)


find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
target_include_directories(ImuPublisherController PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
ament_target_dependencies(
    ImuPublisherController rclcpp sensor_msgs tf2 tf2_ros tf2_geometry_msgs)



choreonoid_add_simple_controller(
    PanTiltController PanTiltController.cpp)
find_package(std_msgs REQUIRED)
ament_target_dependencies(
    PanTiltController rclcpp std_msgs)

