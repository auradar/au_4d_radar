Overview
========

AuRadar4D_ROS2 is a collection of ROS2 packages supporting AU 4D Radar.
This program has been tested on ubuntu 22.04 ROS2 humble.

**Create a new work directory of ROS2**:
```
mkdir -p ~/ros2_ws/src
```

**Download AuRadar4D ROS2 Source code**:
```
cd ~/ros2_ws/src
$ https://github.com/auradar/au_4d_radar.git
```

**Download radar messages**:
```
$ cd ~/ros2_ws/src
$ git clone https://github.com/ros-perception/radar_msgs.git
```

**Download monitor messages**:
```
$ cd ~/ros2_ws/src
$ https://github.com/auradar/mon_msgs.git
```

**Change Target IP**:
```
src/socket.cpp
#define TARGET_IP	"xxx.xxx.xxx.xxx"
```

**Compile & Run**:
```
$ cd ~/ros2_ws
$ colcon build --cmake-args -DCMAKE_BUILD_TYPE=Relaese --packages-up-to au_4d_radar
```

**Run in Terminal 1**:
```
$ cd ~/ros2_ws
ros2 run rclcpp_components component_container
```
**Run in Terminal 2**:
```
$ cd ~/ros2_ws
source install/local_setup.bash
ros2 component load /ComponentManager au_4d_radar au_4d_radar::device_au_radar_node
```
