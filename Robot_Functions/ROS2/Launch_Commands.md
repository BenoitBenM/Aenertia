# ROS 2 Cartographer SLAM Launch Guide

This document describes how to launch a full SLAM stack on ROS 2 **Jazzy** using:

- RPLIDAR A1
- Cartographer
- A custom URDF
- RViz2

---

## 🧰 Prerequisites

- Workspace: `~/gbot_ws`
- Installed and built packages:
  - `mybot_description`
  - `rplidar_ros`
  - `mybot_cartographer`
- Device `/dev/rplidar` used by the LiDAR

---

## Terminal 1 – Launch URDF (robot_state_publisher)

```bash
cd ~/Aenertia/Robot_Functions/ROS2/gbot_ws
source install/setup.bash
ros2 launch mybot_description description.launch.py
```
## Terminal 2 – Launch the LIDAR (robot_state_publisher)
```bash
cd ~/Aenertia/Robot_Functions/ROS2/gbot_ws
source install/setup.bash
ros2 launch rplidar_ros rplidar_a1_launch.py \
  serial_port:=/dev/rplidar \
  serial_baudrate:=115200 \
  frame_id:=laser_frame
```

## Terminal 3 – Launch Launch Cartographer
```bash
cd ~/Aenertia/Robot_Functions/ROS2/gbot_ws
source install/setup.bash
ros2 launch mybot_cartographer carto.launch.py
```
## Terminal 4 – Launch Rviz
```bash
source /opt/ros/jazzy/setup.bash
cd ~/Aenertia/Robot_Functions/ROS2/gbot_ws
source install/setup.bash
ros2 launch mybot_nav bringup.launch.py
```
## Terminal 4 – Launch Rviz
```bash
cd ~/carto_ws
source install/setup.bash
rviz2
```

Once RViz is open:

- Set Fixed Frame to map.

- Click Add and add the following displays:

    - TF
    - LaserScan
    - SubmapsDisplay
    - RobotModel

Configure SubmapsDisplay:
  - Topic: /submap_list
  - Submap query service: /submap_query
  - Tracking frame: base_link


