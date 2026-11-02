cmake_minimum_required(VERSION 3.5)
project(nitro_choreonoid)

## Compile as C++11, supported in ROS2
add_compile_options(-std=c++14)

## Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(choreonoid REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
## Specify additional locations of header files
## Your package locations should be listed before other locations
set(CMAKE_CXX_STANDARD ${CHOREONOID_CXX_STANDARD})
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories(
    include
    ${rclcpp_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${image_transport_INCLUDE_DIRS}
    ${choreonoid_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${tf2_INCLUDE_DIRS}
)


link_directories(${CHOREONOID_LIBRARY_DIRS})

add_subdirectory(src)
## Declare a C++ executable
# add_executable(${PROJECT_NAME}_node src/nitro_choreonoid_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${rclcpp_LIBRARIES}
#   ${std_msgs_LIBRARIES}
#   ${sensor_msgs_LIBRARIES}
#   ${image_transport_LIBRARIES}
#   ${choreonoid_LIBRARIES}
#   ${nav_msgs_LIBRARIES}
#   ${tf2_LIBRARIES}
# )



ament_package()
