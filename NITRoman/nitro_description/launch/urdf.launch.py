from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():

    # ====== ワークスペース src から直接参照 ======
    workspace_src = os.path.join(os.environ["HOME"], "NITRo_WRS2025", "wrs_ws", "src","nitro_simulation")
    packages_name = "nitro_description"
    xacro_file_name = "xacro/nitroman.xacro"
    rviz_file_name = "urdf.rviz"

    # xacro と rviz のパスを絶対パスで指定
    xacro_path = os.path.join(workspace_src, packages_name, xacro_file_name)
    rviz_config_file = os.path.join(workspace_src, packages_name, "rviz", rviz_file_name)

    # URDFをxacroから取得
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            xacro_path,
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # ====== ノード定義 ======
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_pub_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription([
        rviz_node,
        robot_state_pub_node,
        joint_state_pub_gui_node,
    ])
