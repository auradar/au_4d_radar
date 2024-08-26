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
export VERSION=v2.0
cd ~/share/ros2_ws/src
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
$ git clone -b v2.0 https://github.com/auradar/mon_msgs.git
```

**Compile & Run**:
```
$ cd ~/share/ros2_ws
$ colcon build --cmake-args -DCMAKE_BUILD_TYPE=Relaese --packages-up-to au_4d_radar
```

**Running Radar Node**:
```
$ cd ~/share/ros2_ws
source install/local_setup.bash
ros2 launch au_4d_radar run_radar.launch.py
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
In the first shell, start the component container:
$ cd ~/share/ros2_ws
source install/local_setup.bash
ros2 run rclcpp_components component_container
```

**Run in shell 2**:
```
In the second shell load au_4d_radar node
$ cd ~/share/ros2_ws
source install/local_setup.bash
ros2 component load /ComponentManager au_4d_radar au_4d_radar::device_au_radar_node
```

**FlatBuffers Install**:
```
1. Download flatbuf
export VERSION=v24.3.25
git clone https: //github.com/google/flatbuffers.git flatbuffers_$VERSION
cd flatbuffers_$VERSION
git checkout $VERSION
git submodule update --init --recursive

2. Build and install the FlatBuffers
mkdir build
cd build

# Configure to build both static and shared libraries
cmake .. -DCMAKE_BUILD_TYPE         = Release \
      -DFLATBUFFERS_STATIC_FLATC    = ON \
      -DFLATBUFFERS_BUILD_SHAREDLIB = OFF \
      -DFLATBUFFERS_BUILD_CPP17     = ON \
      -DCMAKE_INSTALL_PREFIX        = /usr/local

make -j$(nproc)
sudo make install
```
