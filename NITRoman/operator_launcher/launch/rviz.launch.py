from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


moveit_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        PathJoinSubstitution(
            [FindPackageShare("openarm_ik_tools"), "launch", "openarm_set_from_ik.launch.py"]
        )
    )
)


def generate_launch_description():
    packages_name="operator_launcher"
    urdf_package_name = "nitro_description"
    urdf_file_name = "nitroman.xacro"

    # URDF ファイルのパスを取得
    # urdf_file_path = PathJoinSubstitution(
    #     [FindPackageShare(urdf_package_name), "xacro", urdf_file_name]
    # )
    robot_description = {"robot_description": Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(urdf_package_name), "xacro", urdf_file_name]
            ),
        ]
    )}

    target_robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="target_robot_state_publisher",
        remappings=[
            ("/joint_states", "/target/joint_states"),
            ("/robot_description", "/target/robot_description"),
        ],
        namespace="target",
        parameters=[
            {"frame_prefix": "target/"},
            robot_description, 
        ],
        output="screen",
    )
    
    robot_state_publisher_node= Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        remappings=[
            # ("/joint_states", "/target/joint_states"),
            # ("/robot_description", "/target/robot_description"),
        ],
        parameters=[
            # {"frame_prefix": "target/"},
            robot_description, 
        ],
        output="screen",
    )

    rviz_file_name = "NITRo_UI.rviz"
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(packages_name), "config", rviz_file_name]
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    command_param_file_path = os.path.join(get_package_share_directory('joy2command'), 'config', 'setting.yaml')
    joy_handler_share = get_package_share_directory('joy_handler')
    #PS5コントローラの場合
    yaml_file_path = os.path.join(joy_handler_share, 'config', 'ps5-controller.yaml')
    #ELECOMコントローラの場合
    # yaml_file_path = os.path.join(joy_handler_share, 'config', 'elecom.yaml')
    declare_yaml_arg = DeclareLaunchArgument(
        'yaml_file',
        default_value=yaml_file_path,
        description='Path to the YAML configuration file'
    )
    declare_yaml_velo = DeclareLaunchArgument(
        'config_file',
        default_value= command_param_file_path  , 
        description='Full path to the config file to load parameters from'
    )

    joy = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        output='screen'
    )
    joy_handler = Node(
        package='joy_handler',
        executable='ps5_joy_publisher',
        name='ps5_joy_publisher',
        output='screen',
        parameters=[LaunchConfiguration('yaml_file')]
    )
    joy2command = Node(
        package='joy2command',  
        executable='joy2command',  
        name='joy2command',  
        output='screen',
        parameters=[LaunchConfiguration('config_file')]  
    )
    body_set_from_ik = Node(
        package='openarm_ik_tools',  
        executable='body_set_from_ik',  
        name='body_set_from_ik',  
        output='log',
    )
    flipper_control_node = Node(
        package="flipper_controller",
        executable="flipper_control",
        name="flipper_control",
        remappings=[
            ("/target/joint_states", "/target/joint_states_flipper")
        ],
        output="screen",
    )
    joint_state_node = Node(
        package="nitro_description",
        executable="target_joint_state_publisher",
        name="target_joint_state_publisher",
        output="screen",
    )
    static_transform=Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher",
            output="screen",
            arguments=["0", "0", "0", "0", "0", "0", "base_footprint", "target/base_footprint"]  # x, y, z, roll, pitch, yaw, frame_id, child_frame_id
        )

    nodes = [
        target_robot_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
        declare_yaml_arg,
        declare_yaml_velo,
        moveit_launch,
        joy,
        joy_handler,
        joy2command,
        body_set_from_ik,
        flipper_control_node,
        joint_state_node,
        static_transform
    ]

    return LaunchDescription(nodes)
