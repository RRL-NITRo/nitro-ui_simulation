# openarm_ik_tools

NITRoman の OpenArm（左右アーム）と胴体を、PS5 コントローラから操作するための ROS 2 パッケージです。
コントローラ入力をコマンドに変換し、MoveIt の逆運動学（IK）で目標関節角を計算して publish します。

> **Note**
> このパッケージは **NITRo-UI が独自に作成したもの** です。
> `NITRoman/OpenArm/` ディレクトリに置いていますが、Enactic, Inc. の
> [OpenArm](https://github.com/enactic/openarm) 公式リポジトリには含まれていません。
> OpenArm のパッケージ（`openarm_description`、`openarm_bimanual_moveit_config`）を利用していますが、
> Enactic, Inc. とは関係がなく、推奨も受けていません。

## ノード

| 実行ファイル | 概要 | 主な入力 | 主な出力 |
|---|---|---|---|
| `openarm_joy2commandleft` / `openarm_joy2commandright` | PS5 コントローラ入力を左/右アームのコマンドに変換 | `/ps5_joy` | `/arm_command/{left,right}`, `/arm_twist/{left,right}`, `/gripper_target/{left,right}` |
| `openarm_set_from_ikleft` / `openarm_set_from_ikright` | アームコマンドから MoveIt の IK（`left_arm` / `right_arm` グループ、TCP は `openarm_{left,right}_hand_tcp`）で関節角を計算 | `/arm_command/{left,right}`, `/arm_twist/{left,right}` | `/target/joint_states/arm_{left,right}`, `/calculated_pose/{left,right}` |
| `body_set_from_ik` | 胴体コマンドから胴体の関節角を計算 | `/body_command` | `/target/joint_states/body` |
| `joy2commandflipper` | PS5 コントローラ入力をクローラ速度とフリッパのコマンドに変換 | `/ps5_joy` | `/target/cmd_vel`, `/flipper_command` |

## 依存パッケージ

- ROS 2、MoveIt（`moveit_core`, `moveit_ros_planning`, `moveit_configs_utils`）
- `openarm_description`, `openarm_bimanual_moveit_config`（OpenArm）
- `arm_msgs`, `ps5_msgs`, `flipper_msgs`（本リポジトリ）

## 起動

左右アームの IK ノードを起動します。

```bash
ros2 launch openarm_ik_tools openarm_set_from_ik.launch.py
```

通常は `operator_launcher` の `rviz.launch.py` から起動されます。

## ライセンス

MIT License（リポジトリ直下の [LICENSE](../../../LICENSE) を参照）
