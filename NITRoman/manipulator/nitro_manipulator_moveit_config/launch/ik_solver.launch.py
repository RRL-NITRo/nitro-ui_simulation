import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
import launch
import launch_ros.descriptions
def generate_launch_description():
    # URDF/XACROファイルのパスを取得
    imax_description_path = os.path.join(get_package_share_directory('nitro_description'), 'xacro', 'imax.xacro')
    imax_manipulator_path = get_package_share_directory('nitro_manipulator_moveit_config')
    kinematics_yaml_path = os.path.join(imax_manipulator_path, 'config', 'kinematics.yaml')
    srdf_path = os.path.join(imax_manipulator_path, 'config', 'imax_manipulator.srdf')

    if not os.path.isfile(srdf_path):
        raise FileNotFoundError(f"SRDF file not found at {srdf_path}")
    # Read the SRDF file content
    with open(srdf_path, 'r') as srdf_file:
        srdf_content = srdf_file.read()

    return LaunchDescription([
        # ロボットモデルの引数を宣言
        DeclareLaunchArgument(
            'model',
            default_value=imax_description_path,
            description='Path to the robot model'
        ),
        
        # ロボットの状態をパブリッシュするノード
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher_target',
        #     remappings=[
        #         ('/joint_states', '/target/joint_states')
        #     ],
        #     parameters=[{'robot_description': launch_ros.descriptions.ParameterValue(
        #         launch.substitutions.Command(['xacro ', imax_description_path ]), value_type=str
        #     )}]
        # ),

        # IKソルバーのノード
        Node(
            package='nitro_manipulator_moveit_config',
            executable='nitro_ik_solver',
            name='nitro_ik_solver',
            output='log',
            parameters=[{
                'planning_group_name': 'imax',
                'initial_joint_angles': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 配列に変更
            }],
            remappings=[
                ('/joint_states', '/target/joint_states')
            ]
        ),

        # kinematics.yamlをロードするノード
       
        Node(
            package='ros2param',
            executable='ros2param',
            name='load_kinematics',
            arguments=['load', kinematics_yaml_path]
        ),

        # セマンティックロボット記述をパブリッシュするノード
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher_semantic',
            remappings=[
                ('/joint_states', '/target/joint_states')
            ],
            parameters=[{
                'robot_description_semantic': srdf_content,
                'robot_description': launch_ros.descriptions.ParameterValue(
                launch.substitutions.Command(['xacro ', imax_description_path ]), value_type=str
            )
            }]
        )
    ])
