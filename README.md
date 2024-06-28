Overview
========

AuRadar4D_ROS2 is a collection of ROS2 packages supporting AU 4D Radar.
This program has been tested on ubuntu 22.04 ROS2 humble.

**Download AuRadar4D_ROS2 Source code**:
```
$ git clone https://github.com/auradar/AuRadar4D_ROS2.git
```

**Download radar ROS2 messages**:
```
$ cd AuRadar4D_ROS2
$ git clone https://github.com/ros-perception/radar_msgs.git
```

**Change Target IP**:
```
src/socket.cpp
#define TARGET_IP	"xxx.xxx.xxx.xxx"
```
**Compile & Run**:
```
$ colcon build --cmake-args -DCMAKE_BUILD_TYPE=Relaese
$ source install/local_setup.bash
$ ros2 launch au_4d_radar run_radar.launch.py
```
