from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import Command

def generate_launch_description():
    pkg_share = get_package_share_directory('openarm_description')
    config_share = get_package_share_directory('openarm_bimanual_moveit_config')

    xacro_file = os.path.join(pkg_share, 'urdf', 'robot', 'v10.urdf.xacro')
    robot_description = Command(['xacro ', xacro_file])

    controllers_yaml = os.path.join(config_share, 'config', 'ros2_controllers.yaml')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        )
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
    )
    
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, controllers_yaml],
        output='both'
    )

    # spawner 各種
    spawner_joint_state = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager']
    )

    spawner_left_arm = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_arm_controller', '--controller-manager', '/controller_manager']
    )

    spawner_right_arm = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_arm_controller', '--controller-manager', '/controller_manager']
    )

    spawner_left_gripper = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_gripper_controller', '--controller-manager', '/controller_manager']
    )

    spawner_right_gripper = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_gripper_controller', '--controller-manager', '/controller_manager']
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        control_node,
        spawner_joint_state,
        spawner_left_arm,
        spawner_right_arm,
        spawner_left_gripper,
        spawner_right_gripper
    ])