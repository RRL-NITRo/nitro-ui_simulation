import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # パッケージの共有ディレクトリを取得
    joy_handler_share = get_package_share_directory('joy_handler')

    # YAMLファイルのパスを指定（共有ディレクトリを参照）
    yaml_file_path = os.path.join(joy_handler_share, 'config', 'ps5-controller.yaml')
    command_param_file_path = os.path.join(get_package_share_directory('joy2command'), 'config', 'setting.yaml')

    rviz_file_name = "NITRone.rviz"
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("operator_launcher"), "config", rviz_file_name]
    )

    slam = os.path.join(
        get_package_share_directory('slam_stack'),
        'launch',
        'slam_with_tf_and_pcd.launch.py'
    )

    # urdf_dronepantilt = os.path.join(
    #     get_package_share_directory('nitro_description'),
    #     'launch',
    #     'urdf_dronepantilt.launch.py'
    # )
    
    return LaunchDescription([
        # YAMLファイルのパスを引数として宣言
        DeclareLaunchArgument(
            'yaml_file',
            default_value=yaml_file_path,
            description='Path to the YAML configuration file'
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file],
        ),

        # joyノードを起動
        Node(
            package='joy',
            executable='joy_node',
            name='drone_joy',
            namespace='drone',
            output='screen', 
            remappings=[('/joy', '/drone/joy')]
        ),

        # joy_handlerノードを起動
        Node(
            package='joy_handler',
            executable='ps5_joy_publisher',
            name='drone_ps5_joy_publisher',
            namespace='drone',
            output='screen',
            parameters=[LaunchConfiguration('yaml_file')],
            remappings=[('/ps5_joy', '/drone/ps5_joy'),('/joy', '/drone/joy')]
        ),
        # パラメータファイルのパスを引数として宣言
        DeclareLaunchArgument(
            'config_file',
            default_value= command_param_file_path,  # デフォルトのパスを指定
            description='Full path to the config file to load parameters from'
        ),

        # Nodeの定義
        Node(
            package='joy2command',  # パッケージ名を記入
            executable='joy2command_drone',  # 実行ファイル名
            name='drone_joy2command_drone',  # ノード名
            namespace='drone',
            output='screen',
            parameters=[LaunchConfiguration('config_file')], # パラメータファイルを読み込む
            remappings=[('/ps5_joy', '/drone/ps5_joy')]
        ),

        Node(
            package='joy2command',  # パッケージ名を記入
            executable='PanTilt_command_publisher',  # 実行ファイル名
            name='PanTilt_command_publisher',  # ノード名
            namespace='drone',
            output='screen',
            parameters=[LaunchConfiguration('config_file')], # パラメータファイルを読み込む
            remappings=[('/ps5_joy', '/drone/ps5_joy')]
        ),

        Node(
            package="qr_code_detector",
            executable="qr_code_detector_node",
            name="qr_code_detector",
            output="screen",
        ),

        Node(
            package="qr_code_detector",
            executable="qr_code_switchercopy_node",
            name="qr_code_switcher",
            output="screen",
        ),

        # Node(
        #     package="meter_reader",
        #     executable="meter_reader_node",
        #     name="meter_reader",
        #     output="screen",
        #     remappings=[
        #         ('/simdrone/FrontCamera', '/NITRone/FrontCamera')
        #     ]
        # ),

        Node(
            package='image_recorder',  # パッケージ名を記入
            executable='save_image',  # 実行ファイル名
            name='image_recorder',  # ノード名
            namespace='drone',
            output='screen',
            remappings=[
                ('/ps5_joy', '/drone/ps5_joy'),
                ('/camera/image_raw', '/NITRone/FrontCamera')
            ]
        ),

        # Node(
        #     package='meter_reader_gui',  # パッケージ名を記入
        #     executable='meter_reader_gui',  # 実行ファイル名
        #     name='meter_reader_gui',  # ノード名
        #     output='screen',
        # ),

        Node(
            package='save_map',  # パッケージ名を記入
            executable='save_map',  # 実行ファイル名
            name='save_map',  # ノード名
            namespace='drone',
            output='screen',
            remappings=[
                ('/ps5_joy', '/drone/ps5_joy')
            ]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam)
        )
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(urdf_dronepantilt)
        # ),

    ])
