# tomato_arm_ik

Crane+ の簡易IK，Joy操作，Dynamixel AX MX系への位置指令送信用ROS 2パッケージ．

## Build

```bash
cd ~/colcon_ws/src
git clone https://github.com/Naokita0831/tomato_arm_ik.git
cd ~/colcon_ws
colcon build --packages-select tomato_arm_ik
source install/setup.bash
```

## Joy + RViz

```bash
ros2 launch tomato_arm_ik arm_ik_joy.launch.py 
```

シミュレーション上でcrane+の動きを確認するためのコマンド。Joyによりcrane+を動かすことが可能。

## Joy + Crane_plus + RViz

```bash
ros2 launch tomato_arm_ik crane_joy.launch.py dev:=/dev/ttyUSB0
```

`crane_control` は `joint_states` を購読し，Dynamixel ID 1〜5へ送る．`use_sync_write:=true` 相当の設定でGroupSyncWriteを使い，5軸分を1パケットで送る．
ttyUSB0かどうかは要確認。確認方法は資料を参照

## Joy + MX  +  AX Control
```bash
ros2 run tomato_arm_ik ax_mx_joy --ros-args   -p dev:=/dev/ttyUSB0   -p ax_id:=5   -p mx_id:=10
```

AXとMXを同時に動かすためのコマンドです。コントローラの左スティックを左右に倒すとMXが上下、上下に倒すとAXモータのID５が回転します。

## Joy + MX  + Crane_plus + RViz
```bash
ros2 launch tomato_arm_ik crane_mx_joy.launch.py dev:=/dev/ttyUSB0
```

Crane+とMxサーボを同時に動かすためのコマンド。
