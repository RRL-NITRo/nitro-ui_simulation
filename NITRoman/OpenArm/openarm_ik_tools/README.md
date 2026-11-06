# openarm_ik_tools

A ROS 2 package for operating the OpenArm arms (left and right) and the body of NITRoman with a PS5 controller.
It converts controller input into commands, computes target joint angles with MoveIt inverse kinematics (IK), and publishes them.

> **Note**
> This package was **written by NITRo-UI**.
> It is placed in the `NITRoman/OpenArm/` directory, but it is not part of the official
> [OpenArm](https://github.com/enactic/openarm) repositories by Enactic, Inc.
> It uses OpenArm packages (`openarm_description`, `openarm_bimanual_moveit_config`),
> but this project is not affiliated with or endorsed by Enactic, Inc.

## Nodes

| Executable | Description | Main inputs | Main outputs |
|---|---|---|---|
| `openarm_joy2commandleft` / `openarm_joy2commandright` | Converts PS5 controller input into commands for the left / right arm | `/ps5_joy` | `/arm_command/{left,right}`, `/arm_twist/{left,right}`, `/gripper_target/{left,right}` |
| `openarm_set_from_ikleft` / `openarm_set_from_ikright` | Computes joint angles from arm commands using MoveIt IK (`left_arm` / `right_arm` group, TCP: `openarm_{left,right}_hand_tcp`) | `/arm_command/{left,right}`, `/arm_twist/{left,right}` | `/target/joint_states/arm_{left,right}`, `/calculated_pose/{left,right}` |
| `body_set_from_ik` | Computes body joint angles from body commands | `/body_command` | `/target/joint_states/body` |
| `joy2commandflipper` | Converts PS5 controller input into crawler velocity and flipper commands | `/ps5_joy` | `/target/cmd_vel`, `/flipper_command` |

## Dependencies

- ROS 2, MoveIt (`moveit_core`, `moveit_ros_planning`, `moveit_configs_utils`)
- `openarm_description`, `openarm_bimanual_moveit_config` (OpenArm)
- `arm_msgs`, `ps5_msgs`, `flipper_msgs` (this repository)

## Launch

Start the IK nodes for the left and right arms:

```bash
ros2 launch openarm_ik_tools openarm_set_from_ik.launch.py
```

Usually these nodes are started from `rviz.launch.py` in `operator_launcher`.

## License

MIT License (see [LICENSE](../../../LICENSE) at the repository root)
