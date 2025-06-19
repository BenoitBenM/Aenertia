## Ubuntu terminal vs GUI mode

### Terminal mode

Permanent
```bash
sudo systemctl set-default multi-user.target
```

Temporary
```bash
sudo systemctl isolate multi-user.target
```


### GUI mode

Temporary
```bash
sudo systemctl isolate graphical.target
```

Permanent
```bash
sudo systemctl set-default graphical.target
```
## Power saving

```bash
./low_power_setup.sh

cd Aenertia/Robot_Functions/ROS2/gbot_ws/src/mybot_nav/mybot_nav/
nice -n 19 python3 mqtt_serial_bridge.py
```

```bash
cd ~/Aenertia/Robot_Functions/ROS2/gbot_ws
colcon build --packages-select mybot_nav
source install/setup.bash
ros2 launch mybot_nav bringup.launch.py
```
