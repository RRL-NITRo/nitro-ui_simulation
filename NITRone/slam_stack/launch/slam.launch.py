import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('slam_stack')
    rtabmap_params = os.path.join(pkg_share, 'config', 'rtabmap_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    deskewing = LaunchConfiguration('deskewing')
    voxel_leaf_size = LaunchConfiguration('voxel_leaf_size')
    input_topic = LaunchConfiguration('input_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    assembled_cloud_topic = LaunchConfiguration('assembled_cloud_topic')
    frame_id = LaunchConfiguration('frame_id')
    odom_frame_id = LaunchConfiguration('odom_frame_id')
    map_frame_id = LaunchConfiguration('map_frame_id')
    expected_update_rate = LaunchConfiguration('expected_update_rate')
    assembler_max_clouds = LaunchConfiguration('assembler_max_clouds')
    enable_viz = LaunchConfiguration('enable_viz')
    odom_topic = LaunchConfiguration('odom_topic')
    odom_info_topic = LaunchConfiguration('odom_info_topic')

    declarations = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulated clock if true.'),
        DeclareLaunchArgument(
            'deskewing', default_value='false',
            description='Enable LiDAR deskewing inside icp_odometry.'),
        DeclareLaunchArgument(
            'voxel_leaf_size', default_value='0.0',
            description='VoxelGrid leaf size for cloud_repacker (0.0 to disable).'),
        DeclareLaunchArgument(
            'input_topic', default_value='/NITRone/RangeSensor/point_cloud',
            description='Raw LiDAR PointCloud2 topic.'),
        DeclareLaunchArgument(
            'scan_topic', default_value='/scan_cloud',
            description='Normalized point cloud topic published by cloud_repacker.'),
        DeclareLaunchArgument(
            'assembled_cloud_topic', default_value='/assembled_cloud',
            description='Accumulated cloud topic consumed by RTAB-Map.'),
        DeclareLaunchArgument(
            'frame_id', default_value='Root',
            description='Robot base frame.'),
        DeclareLaunchArgument(
            'odom_frame_id', default_value='odom',
            description='Odometry frame published by icp_odometry.'),
        DeclareLaunchArgument(
            'map_frame_id', default_value='map',
            description='Global map frame published by RTAB-Map.'),
        DeclareLaunchArgument(
            'expected_update_rate', default_value='15.0',
            description='Expected LiDAR frame rate for icp_odometry.'),
        DeclareLaunchArgument(
            'assembler_max_clouds', default_value='10',
            description='Number of recent scans to keep in point_cloud_assembler.'),
        DeclareLaunchArgument(
            'enable_viz', default_value='true',
            description='Launch rtabmap_viz when true.'),
        DeclareLaunchArgument(
            'odom_topic', default_value='/slam_stack/icp_odom',
            description='Odometry topic name published by icp_odometry.'),
        DeclareLaunchArgument(
            'odom_info_topic', default_value='/slam_stack/icp_odom_info',
            description='Odometry info topic name published by icp_odometry.'),
    ]

    cloud_repacker_node = Node(
        package='slam_stack',
        executable='cloud_repacker',
        name='cloud_repacker',
        output='screen',
        parameters=[{
            'input': input_topic,
            'output': scan_topic,
            'voxel_leaf_size': ParameterValue(voxel_leaf_size, value_type=float),
        }]
    )

    icp_odometry_node = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        output='screen',
        parameters=[{
            'frame_id': frame_id,
            'odom_frame_id': odom_frame_id,
            'publish_tf': True,
            'wait_for_transform': 0.2,
            'expected_update_rate': ParameterValue(expected_update_rate, value_type=float),
            'deskewing': ParameterValue(deskewing, value_type=bool),
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'Icp/PointToPlane': 'true',
            'Icp/Iterations': '10',
            'Icp/VoxelSize': '0.1',
            'Icp/Epsilon': '0.001',
            'Icp/PointToPlaneK': '20',
            'Icp/PointToPlaneRadius': '0',
            'Icp/MaxTranslation': '2',
            'Icp/MaxCorrespondenceDistance': '1',
            'Icp/Strategy': '1',
            'Icp/OutlierRatio': '0.7',
            'Icp/CorrespondenceRatio': '0.01',
            'Odom/ScanKeyFrameThr': '0.4',
            'OdomF2M/ScanSubtractRadius': '0.1',
            'OdomF2M/ScanMaxSize': '15000',
            'OdomF2M/BundleAdjustment': 'false'
        }],
        remappings=[
            ('scan_cloud', scan_topic),
            ('odom', odom_topic),
            ('odom_info', odom_info_topic),
        ]
    )

    point_cloud_assembler_node = Node(
        package='rtabmap_util',
        executable='point_cloud_assembler',
        name='point_cloud_assembler',
        output='screen',
        parameters=[{
            'max_clouds': ParameterValue(assembler_max_clouds, value_type=int),
            'fixed_frame_id': odom_frame_id,
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
        remappings=[
            ('cloud', 'odom_filtered_input_scan'),
            ('assembled_cloud', assembled_cloud_topic),
        ]
    )

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            rtabmap_params,
            {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'frame_id': frame_id,
                'odom_frame_id': odom_frame_id,
                'map_frame_id': map_frame_id,
            },
        ],
        remappings=[
            ('odom', odom_topic),
            ('odom_info', odom_info_topic),
            ('scan_cloud', assembled_cloud_topic),
        ],
        arguments=['-d']
    )

    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        condition=IfCondition(enable_viz),
        parameters=[{
            'frame_id': frame_id,
            'odom_frame_id': odom_frame_id,
            'subscribe_odom_info': True,
            'subscribe_scan_cloud': True,
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
        remappings=[
            ('odom', odom_topic),
            ('odom_info', odom_info_topic),
            ('scan_cloud', assembled_cloud_topic),
        ]
    )

    nodes = [
        cloud_repacker_node,
        icp_odometry_node,
        point_cloud_assembler_node,
        rtabmap_node,
        rtabmap_viz_node,
    ]

    return LaunchDescription(declarations + nodes)
