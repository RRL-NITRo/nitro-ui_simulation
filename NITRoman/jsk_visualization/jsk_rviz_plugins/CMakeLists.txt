cmake_minimum_required(VERSION 3.5)
project(jsk_rviz_plugins)

# Default to C99
if (NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)

# for Ogre include files
include_directories(${ADDITIONAL_INCLUDE_DIRS} ${OGRE_INCLUDE_DIRS} ${OGRE_INCLUDE_DIRS}/Paging)

# Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

add_library(jsk_rviz_plugins SHARED
  ### Display
  src/overlay_utils.cpp
  src/string_display.cpp)

# Link ament packages
ament_target_dependencies(jsk_rviz_plugins
  rclcpp
  rviz_common
  rviz_rendering
  std_msgs
  pluginlib
  )

if (NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
  )
endif()

# Link non ament packages
target_link_libraries(jsk_rviz_plugins Qt5::Widgets)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

# replaces catkin_package(LIBRARIES jsk_rviz_plugins)
ament_export_libraries(jsk_rviz_plugins)

ament_export_targets(export_jsk_rviz_plugins HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rviz_common
  rviz_rendering
  rviz_ogre_vendor
  rclcpp
  std_msgs
)

install(TARGETS
  jsk_rviz_plugins
  EXPORT export_jsk_rviz_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

ament_package()
