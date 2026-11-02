import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Path to your URDF file
    robot_urdf_path = get_package_share_directory('openarm_description') + '/urdf/robot/v10.urdf.xacro'

    # Load the URDF into the robot_description parameter
    robot_description = open(robot_urdf_path, 'r').read()
    moveit_config = MoveItConfigsBuilder("openarm", package_name="openarm_bimanual_moveit_config").to_moveit_configs()

    moveit_nodeleft=Node(
        package='openarm_ik_tools',
        executable='openarm_set_from_ikleft',  # Change to your actual executable name
        output='screen',
        namespace="leftarm",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    moveit_noderight=Node(
        package='openarm_ik_tools',
        executable='openarm_set_from_ikright',  # Change to your actual executable name
        output='screen',
        namespace="rightarm",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )


    return LaunchDescription([
        # robot_state_publisher_nodeleft, 
        moveit_nodeleft, 
        # robot_state_publisher_noderight, 
        moveit_noderight])