Overview
========

au_4d_radar is a collection of ROS2 packages supporting AU 4D Radar.
This program has been tested on ubuntu 22.04 ROS2 humble.

**Create a new work directory of ROS2**:
```
mkdir -p ~/share/ros2_ws/src
```

**Download AuRadar4D ROS2 Source code**:
```
$ cd ~/share/ros2_ws/src
$ export VERSION=v1.0
$ git clone https://github.com/auradar/au_4d_radar.git
$ cd au_4d_radar
$ git checkout $VERSION
$ git submodule update --init --recursive
```

**Download radar messages**:
```
$ cd ~/share/ros2_ws/src
$ git clone https://github.com/ros-perception/radar_msgs.git
```

**Download monitor messages**:
```
$ cd ~/share/ros2_ws/src
$ git clone https://github.com/auradar/mon_msgs.git
```

**Change Target IP**:
```
src/socket.cpp
#define TARGET_IP	"xxx.xxx.xxx.xxx"
```

**Compile**:
```
$ cd ~/share/ros2_ws
$ colcon build --cmake-args -DCMAKE_BUILD_TYPE=Relaese --packages-up-to au_4d_radar
```

**Running Radar Node using launch actions**:
```
$ cd ~/share/ros2_ws
source install/local_setup.bash
ros2 launch au_4d_radar run_radar.launch.py
```

Run-time composition using ROS services
=============

**Run in shell 1**:
```
$ cd ~/share/ros2_ws
source install/local_setup.bash
ros2 run rclcpp_components component_container
```
**Run in shell 2**:
```
$ cd ~/share/ros2_ws
source install/local_setup.bash
ros2 component load /ComponentManager au_4d_radar au_4d_radar::device_au_radar_node
```
