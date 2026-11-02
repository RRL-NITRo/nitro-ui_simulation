import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Path to your URDF file
    robot_urdf_path = get_package_share_directory('nitro_description') + '/xacro/imax.urdf'
    
    # Load the URDF into the robot_description parameter
    robot_description = open(robot_urdf_path, 'r').read()
    moveit_config = MoveItConfigsBuilder("imax", package_name="nitro_manipulator_moveit_config").to_moveit_configs()
     
    moveit_node=Node(
        package='nitro_manipulator_moveit_config',
        executable='set_from_ik',  # Change to your actual executable name
        output='screen',
        remappings=[
            ('/target/joint_states', '/target/joint_states_arm')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        remappings=[
            ('/joint_states', '/target/joint_states'),
            ('/robot_description', '/target/robot_description')
        ],
        parameters=[{'robot_description': robot_description},
                    {'frame_prefix': "target/"}]
    )

    return LaunchDescription([robot_state_publisher_node, moveit_node])
 