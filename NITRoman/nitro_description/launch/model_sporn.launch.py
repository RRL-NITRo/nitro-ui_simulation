import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
  nitro_description_path = get_package_share_directory('nitro_description')
  gazebo_ros_path = get_package_share_directory('gazebo_ros')
  model_path = os.path.join(nitro_description_path, 'xacro', 'imax_manipulator.xacro')

  return LaunchDescription([
    DeclareLaunchArgument(
      'model',
      default_value=model_path,
      description='Path to the robot model'
    ),
    Node(
      package='xacro',
      executable='xacro',
      arguments=[model_path],
      output='screen',
      name='robot_description'
    ),
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        os.path.join(gazebo_ros_path, 'launch', 'empty_world.launch.py')
      )
    ),
    Node(
      package='gazebo_ros',
      executable='spawn_entity.py',
      name='urdf_spawner',
      output='screen',
      arguments=['-entity', 'imax', '-topic', 'robot_description']
    )
  ])