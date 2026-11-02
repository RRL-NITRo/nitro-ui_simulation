import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('slam_stack')
    slam_with_tf_launch = os.path.join(pkg_share, 'launch', 'slam_with_tf.launch.py')

    home_dir = os.environ.get('HOME', '')
    default_pcd_dir = os.path.join(home_dir, 'pcd') if home_dir else '.'

    common_arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulated clock if true.'),
        DeclareLaunchArgument('deskewing', default_value='false', description='Enable LiDAR deskewing.'),
        DeclareLaunchArgument('voxel_leaf_size', default_value='0.0', description='VoxelGrid leaf size for cloud_repacker.'),
        DeclareLaunchArgument('input_topic', default_value='/NITRone/RangeSensor/point_cloud', description='Raw LiDAR topic.'),
        DeclareLaunchArgument('scan_topic', default_value='/scan_cloud', description='Normalized scan topic.'),
        DeclareLaunchArgument('assembled_cloud_topic', default_value='/assembled_cloud', description='Accumulated scan topic.'),
        DeclareLaunchArgument('frame_id', default_value='Root', description='Robot base frame.'),
        DeclareLaunchArgument('sensor_frame', default_value='RangeSensor', description='LiDAR sensor frame.'),
        DeclareLaunchArgument('odom_frame_id', default_value='odom', description='Odometry frame.'),
        DeclareLaunchArgument('map_frame_id', default_value='map', description='Map frame.'),
        DeclareLaunchArgument('expected_update_rate', default_value='15.0', description='Expected LiDAR frame rate.'),
        DeclareLaunchArgument('assembler_max_clouds', default_value='10', description='History size for point_cloud_assembler.'),
        DeclareLaunchArgument('enable_viz', default_value='true', description='Launch rtabmap_viz when true.'),
        DeclareLaunchArgument('odom_topic', default_value='/slam_stack/icp_odom', description='Odometry topic name.'),
        DeclareLaunchArgument('odom_info_topic', default_value='/slam_stack/icp_odom_info', description='Odometry info topic name.'),
        DeclareLaunchArgument('sensor_x', default_value='0.0', description='Sensor origin offset X (m).'),
        DeclareLaunchArgument('sensor_y', default_value='0.0', description='Sensor origin offset Y (m).'),
        DeclareLaunchArgument('sensor_z', default_value='0.0', description='Sensor origin offset Z (m).'),
        DeclareLaunchArgument('sensor_qx', default_value='0.0', description='Sensor orientation quaternion X.'),
        DeclareLaunchArgument('sensor_qy', default_value='0.0', description='Sensor orientation quaternion Y.'),
        DeclareLaunchArgument('sensor_qz', default_value='0.0', description='Sensor orientation quaternion Z.'),
        DeclareLaunchArgument('sensor_qw', default_value='1.0', description='Sensor orientation quaternion W.'),
    ]

    pcd_arguments = [
        DeclareLaunchArgument('enable_pcd_writer', default_value='true', description='Launch the PCD writer when true.'),
        DeclareLaunchArgument('pcd_input_topic', default_value='/cloud_map', description='PointCloud2 topic to save.'),
        DeclareLaunchArgument('pcd_directory', default_value=default_pcd_dir, description='Directory where PCD files are written.'),
        DeclareLaunchArgument('pcd_file_prefix', default_value='cloud_map', description='Filename prefix for saved clouds.'),
        DeclareLaunchArgument('pcd_binary', default_value='false', description='Write binary PCD files when true.'),
        DeclareLaunchArgument('pcd_save_once', default_value='false', description='Stop after the first save when true.'),
        DeclareLaunchArgument('pcd_start_index', default_value='0', description='Initial index for generated filenames.'),
    ]

    slam_with_tf_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_with_tf_launch),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'deskewing': LaunchConfiguration('deskewing'),
            'voxel_leaf_size': LaunchConfiguration('voxel_leaf_size'),
            'input_topic': LaunchConfiguration('input_topic'),
            'scan_topic': LaunchConfiguration('scan_topic'),
            'assembled_cloud_topic': LaunchConfiguration('assembled_cloud_topic'),
            'frame_id': LaunchConfiguration('frame_id'),
            'sensor_frame': LaunchConfiguration('sensor_frame'),
            'odom_frame_id': LaunchConfiguration('odom_frame_id'),
            'map_frame_id': LaunchConfiguration('map_frame_id'),
            'expected_update_rate': LaunchConfiguration('expected_update_rate'),
            'assembler_max_clouds': LaunchConfiguration('assembler_max_clouds'),
            'enable_viz': LaunchConfiguration('enable_viz'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'odom_info_topic': LaunchConfiguration('odom_info_topic'),
            'sensor_x': LaunchConfiguration('sensor_x'),
            'sensor_y': LaunchConfiguration('sensor_y'),
            'sensor_z': LaunchConfiguration('sensor_z'),
            'sensor_qx': LaunchConfiguration('sensor_qx'),
            'sensor_qy': LaunchConfiguration('sensor_qy'),
            'sensor_qz': LaunchConfiguration('sensor_qz'),
            'sensor_qw': LaunchConfiguration('sensor_qw'),
        }.items()
    )

    pcd_writer_node = Node(
        package='slam_stack',
        executable='pcd_writer',
        name='pcd_writer',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_pcd_writer')),
        parameters=[{
            'input_topic': LaunchConfiguration('pcd_input_topic'),
            'directory': LaunchConfiguration('pcd_directory'),
            'file_prefix': LaunchConfiguration('pcd_file_prefix'),
            'binary': ParameterValue(LaunchConfiguration('pcd_binary'), value_type=bool),
            'save_once': ParameterValue(LaunchConfiguration('pcd_save_once'), value_type=bool),
            'start_index': ParameterValue(LaunchConfiguration('pcd_start_index'), value_type=int),
        }]
    )

    return LaunchDescription(common_arguments + pcd_arguments + [slam_with_tf_include, pcd_writer_node])
