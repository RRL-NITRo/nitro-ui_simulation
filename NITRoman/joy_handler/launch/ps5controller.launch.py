import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # パッケージの共有ディレクトリを取得
    joy_handler_share = get_package_share_directory('joy_handler')

    # YAMLファイルのパスを指定（共有ディレクトリを参照）
    yaml_file_path = os.path.join(joy_handler_share, 'config', 'ps5-controller.yaml')

    return LaunchDescription([
        # YAMLファイルのパスを引数として宣言
        DeclareLaunchArgument(
            'yaml_file',
            default_value=yaml_file_path,
            description='Path to the YAML configuration file'
        ),

        # joyノードを起動
        Node(
            package='joy',
            executable='joy_node',
            name='joy',
            output='screen'
        ),

        # joy_handlerノードを起動
        Node(
            package='joy_handler',
            executable='ps5_joy_publisher',
            name='ps5_joy_publisher',
            output='screen',
            parameters=[LaunchConfiguration('yaml_file')]
        )
    ])
