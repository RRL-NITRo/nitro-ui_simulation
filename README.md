# nitro-ui_simulation

ROS 2 packages for simulating and operating NITRo-UI's robots in Choreonoid:
**NITRoman** (a crawler robot with dual arms) and **NITRone** (a drone).

- ROS 2 Humble
- Choreonoid 2.x

## NITRoman

| Package | License | Description |
|---|---|---|
| [nitro_description](NITRoman/nitro_description) | Apache-2.0 | Robot model of NITRoman (xacro / meshes), including the arm model derived from OpenArm |
| [nitro_choreonoid](NITRoman/nitro_choreonoid) | MIT | Choreonoid controllers (crawler, body, left/right arm joint control, joint state publishing) |
| [joy_handler](NITRoman/joy_handler) | MIT | Converts input from various game controllers (PS5, Switch Pro Controller) into common topics. Key bindings are configurable |
| [joy2command](NITRoman/joy2command) | MIT | Converts controller input into robot commands (arms, drone, pan-tilt) |
| [ps5_msgs](NITRoman/ps5_msgs) | MIT | Message definitions for PS5 controller input |
| [arm_msgs](NITRoman/manipulator/arm_msgs) | MIT | Message definitions for arm and body commands |
| [flipper_controller](NITRoman/flipper/flipper_controller) | MIT | Computes and publishes target flipper joint angles from flipper commands |
| [flipper_msgs](NITRoman/flipper/flipper_msgs) | MIT | Message definitions for flipper commands |
| [openarm_ik_tools](NITRoman/OpenArm/openarm_ik_tools) | MIT | Computes target joint angles for OpenArm (left/right arms) and the body using MoveIt IK. Also converts controller input into arm commands |
| [operator_launcher](NITRoman/operator_launcher) | MIT | Launch files and RViz configs for operation (for NITRoman and NITRone) |
| [jsk_rviz_plugins](NITRoman/jsk_visualization/jsk_rviz_plugins) | BSD | RViz plugins (ROS 2 port of JSK Lab's jsk_visualization) |

## NITRone

| Package | License | Description |
|---|---|---|
| [nitro_choreonoid_ros2_sample_drone_tutorial](NITRone/nitro_choreonoid_ros2_sample_drone_tutorial) | MIT / GPL-3.0 | Choreonoid controllers for the drone (flight, pan-tilt camera, IMU). Modified from a sample by Kenta Suzuki. Distributed as a whole under GPL-3.0 because it includes the GPL-3.0 MadgwickAHRS code |
| [slam_stack](NITRone/slam_stack) | MIT | Launch stack for LiDAR SLAM using RTAB-Map |
| [save_map](NITRone/save_map) | MIT | Saves the SLAM point cloud map (`/rtabmap/cloud_map`) to a PCD file |
| [image_recorder](NITRone/image_recorder) | MIT | Saves camera images to files |
| [qr_code_detector](NITRone/qr_code_detector) | MIT | Detects QR codes in images and publishes images annotated with the decoded text |

## Choreonoid Models

| Directory | License | Description |
|---|---|---|
| [choreonoid_model_NITRoUI](choreonoid_model_NITRoUI) | MIT (Apache-2.0 for `nitromanlight.body` only) | Choreonoid models (`.body`) and project files (`.cnoid`) for NITRoman and NITRone. Copy this directory into Choreonoid's `ext` directory and build |

## About OpenArm

The arms of NITRoman use [OpenArm](https://github.com/enactic/openarm),
an open-source robot arm published by Enactic, Inc.
This repository includes copies of the following OpenArm packages (all Apache-2.0).

| Package | License | Description |
|---|---|---|
| [openarm_can](NITRoman/OpenArm/openarm_can) | Apache-2.0 | Library for controlling OpenArm motors over CAN ([upstream](https://github.com/enactic/openarm_can)) |
| [openarm_description](NITRoman/OpenArm/openarm_description) | Apache-2.0 | URDF / xacro model and meshes of OpenArm ([upstream](https://github.com/enactic/openarm_description)) |
| [openarm_ros2](NITRoman/OpenArm/openarm_ros2) | Apache-2.0 | ROS 2 packages for OpenArm (hardware interface, bringup, MoveIt configs) ([upstream](https://github.com/enactic/openarm_ros2)) |

The following files are derived from `openarm_description` and were modified by NITRo-UI (Apache-2.0):

- `NITRoman/nitro_description/xacro/openarm.xacro` (integrated into NITRoman; meshes and colors changed)
- `NITRoman/nitro_description/meshes/arm/v10/`, `NITRoman/nitro_description/meshes/ee/openarm_hand/` (lightweight / simplified meshes)
- `choreonoid_model_NITRoUI/model/nitromanlight.body` (converted to the Choreonoid format)

Note that `NITRoman/OpenArm/openarm_ik_tools` was written by NITRo-UI and is not part of the official OpenArm packages.

This project is not affiliated with or endorsed by Enactic, Inc.
The name "OpenArm" is used only to identify the origin of the software used here.

## License

Unless otherwise noted, this repository is licensed under the [MIT License](LICENSE).
Some packages and files are under other licenses (Apache-2.0, BSD, GPL-3.0), as shown in the tables above.
See [NOTICE](NOTICE) for third-party copyright notices and the list of modified files.
