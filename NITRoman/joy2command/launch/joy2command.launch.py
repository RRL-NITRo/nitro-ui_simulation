from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    yaml_file_path = os.path.join(get_package_share_directory('joy2command'), 'config', 'setting.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value= yaml_file_path , 
            description='Full path to the config file to load parameters from'
        ),

        # Nodeの定義
        Node(
            package='joy2command', 
            executable='joy2command',  
            name='joy2command',  
            output='screen',
            parameters=[LaunchConfiguration('config_file')]  
        ),
    ])
