## 终端 A 跑 ORB SLAM3 单目 C++ 节点

新开终端 A

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ./install/setup.bash
ros2 run ros2_orb_slam3 mono_node_cpp --ros-args -p node_name_arg:=mono_slam_cpp
```

保持这个终端开着

## 3 终端 B 跑 EuRoC MH05 示例驱动

新开终端 B

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ./install/setup.bash
ros2 run ros2_orb_slam3 mono_driver_node.py --ros-args -p settings_name:=EuRoC -p image_seq:=sample_euroc_MH05
```

正常情况下，两个节点会先 handshake，然后开始发布图像并进入 tracking

## 4 如果还是可以直接按报错逐条给您改到能跑。







## 1 先用最小化命令跑您的数据

先把 publish_rate_hz 去掉，和 EuRoC 保持一致的形态。

终端 A 先开着 mono_node_cpp
终端 B 用这个

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ./install/setup.bash
ros2 run ros2_orb_slam3 mono_driver_node.py --ros-args \
  -p settings_name:=MYLEFT \
  -p image_seq:=left_data/left_standard_2025_11_25-13_20_22
```