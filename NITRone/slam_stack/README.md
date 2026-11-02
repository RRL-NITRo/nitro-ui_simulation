# slam_stack

ROS 2 Humble launch stack that mirrors the `rtabmap_livox` setup for the SampleDrone
simulation. It normalizes Choreonoid LiDAR scans, estimates odometry with
`rtabmap_odom/icp_odometry`, accumulates scans, and feeds them to
`rtabmap_slam/rtabmap`.

## Dependencies

Install `rtabmap_ros` (and underlying `rtabmap` / `rtabmap_viz`) from binaries:

```bash
sudo apt update
sudo apt install ros-humble-rtabmap-ros
```

## Build

```bash
cd ~/inSectWRS2025/wrs_ws
bash ~/inSectWRS2025/script/10_ws_build.sh
```

## Launch

### Without static TF

```bash
ros2 launch slam_stack slam.launch.py use_sim_time:=true
```

### With static TF (base_link -> sensor)

```bash
ros2 launch slam_stack slam_with_tf.launch.py use_sim_time:=true
```

`slam.launch.py` exposes the main parameters:

- `input_topic` (default `/SampleDrone/os_sensor/point_cloud`)
- `scan_topic` (normalized cloud, default `/scan_cloud`)
- `assembled_cloud_topic` (default `/assembled_cloud`)
- `frame_id` / `odom_frame_id` / `map_frame_id`
- `voxel_leaf_size`, `deskewing`, `expected_update_rate`, `assembler_max_clouds`
- `odom_topic` / `odom_info_topic` (`icp_odometry` outputs, default `/slam_stack/icp_odom(_info)`).

`slam_with_tf.launch.py` adds a `tf2_ros/static_transform_publisher` for
`SampleDrone/base_link -> os_sensor` and forwards the same arguments.

## Topics (default)

```
/SampleDrone/os_sensor/point_cloud  # raw Choreonoid LiDAR
/scan_cloud                         # normalized XYZ cloud from cloud_repacker
/slam_stack/icp_odom                # odometry from icp_odometry
/slam_stack/icp_odom_info           # odometry diagnostic info
/assembled_cloud                    # accumulated scans for rtabmap
/rtabmap/cloud_map                  # RTAB-Map global point cloud (RViz)
```

Ensure TF tree connects `map -> odom -> SampleDrone/base_link -> os_sensor`.
`slam_with_tf.launch.py` publishes the static `base_link -> os_sensor` transform;
`icp_odometry` publishes `odom -> base_link`; `rtabmap` provides `map -> odom`.

## Exporting Point Clouds

`pcd_writer` subscribes to any `sensor_msgs/PointCloud2` and saves sequential PCD files:

```bash
ros2 run slam_stack pcd_writer --ros-args \
  -p input_topic:=/cloud_map \
  -p directory:=/tmp/pcd \
  -p file_prefix:=cloud_map \
  -p binary:=false \
  -p save_once:=false
```

- Set `save_once:=true` to exit after the first cloud.
- Ensure the output directory already exists (files are named `<prefix>_0000.pcd`, etc.).
