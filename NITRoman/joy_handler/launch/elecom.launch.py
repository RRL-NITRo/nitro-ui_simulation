import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # joy_handler パッケージの share ディレクトリを取得
    pkg_share = get_package_share_directory('joy_handler')

    # config/elecom.yaml の絶対パス
    yaml_file_path = os.path.join(pkg_share, 'config', 'elecom.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'yaml_file',
            default_value=yaml_file_path,
            description='Path to the YAML configuration file'
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy',
            output='screen'
        ),
        Node(
            package='joy_handler',
            executable='ps5_joy_publisher',
            name='ps5_joy_publisher',
            output='screen',
            parameters=[LaunchConfiguration('yaml_file')]
        )
    ])
