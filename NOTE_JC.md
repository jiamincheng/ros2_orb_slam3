**Terminal A runs the ORB SLAM3 node**

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run ros2_orb_slam3 mono_node_cpp
```

**Terminal B runs the Python driver**

If your data is in
`~/ros2_ws/src/ros2_orb_slam3/TEST_DATASET/left_data/left_standard_2025_11_25-13_20_22/`

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run ros2_orb_slam3 mono_driver_node.py --ros-args \
  -p settings_name:=MYLEFT \
  -p image_seq:=left_data/left_standard_2025_11_25-13_20_22
```

This driver publishes
`/mono_py_driver/img_msg` image messages
`/mono_py_driver/timestep_msg` timestamps
It also performs a handshake with the ORB node.

**Terminal C runs the caustics mask node**

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/caustics_mask/caustics_mask/caustics_mask_node.py --ros-args \
  -p image_topic:=/mono_py_driver/img_msg \
  -p mask_topic:=/caustics/mask
```

It publishes `/caustics/mask`
White 255 means keep this region
Black 0 means caustics region.

**Terminal D runs the viewer**

It subscribes to `/caustics/mask` and shows the mask in a window.

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/caustics_mask/caustics_mask/mask_viewer.py
```