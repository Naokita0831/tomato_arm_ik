# tomato_arm_ik ROS 2

Crane+ の簡易IK，Joy操作，Dynamixel AX系への位置指令送信用ROS 2パッケージ．

## Build

```bash
cd ~/colcon_ws/src
unzip tomato_arm_ik_ros2.zip
cd ~/colcon_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select tomato_arm_ik
source install/setup.bash
```

## RViz + Joy only

```bash
ros2 launch tomato_arm_ik arm_ik_joy.launch.py
```

## Joy + Dynamixel

```bash
ros2 launch tomato_arm_ik crane_joy.launch.py dev:=/dev/ttyUSB0
```

`crane_control` は `joint_states` を購読し，Dynamixel ID 1〜5へ送る．`use_sync_write:=true` 相当の設定でGroupSyncWriteを使い，5軸分を1パケットで送る．
