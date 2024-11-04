AU 4D Radar
===========

## Introduction

au_4d_radar is a collection of ROS2 packages supporting 4D Radar Sensors.
This program has been tested on ubuntu 22.04 ROS2 humble.
Supports automatic connection and communication functions without having to set the IP of each Radar sensor in a local network environment.

## How to build the au_4d_radar

**Create a new work directory of ROS2**:

```bash
mkdir -p ~/share/ros2_ws/src
```

**Download AU 4D Radar ROS2 Source code**:

```bash
$ export VERSION=v2.x
$ cd ~/share/ros2_ws/src
$ git clone https://github.com/auradar/au_4d_radar.git
$ cd au_4d_radar
$ git checkout $VERSION
$ git submodule update --init --recursive
```

**Download radar messages**:

```bash
$ cd ~/share/ros2_ws/src
$ git clone https://github.com/ros-perception/radar_msgs.git
```

**Download monitor messages**:

```bash
$ cd ~/share/ros2_ws/src
$ git clone -b v2.0 https://github.com/auradar/mon_msgs.git
```

### Install FlatBuffers:

**1. Download the FlatBuffers**

```bash
export VERSION=v24.3.25
git clone https://github.com/google/flatbuffers.git flatbuffers_$VERSION
cd flatbuffers_$VERSION
git checkout $VERSION
git submodule update --init --recursive
```

**2. Build and install the FlatBuffers**

```cmake
mkdir build
cd build

# Configure to build both static and shared libraries
cmake .. -DCMAKE_BUILD_TYPE=Release \
      -DFLATBUFFERS_STATIC_FLATC=ON \
      -DFLATBUFFERS_BUILD_SHAREDLIB=OFF \
      -DFLATBUFFERS_BUILD_CPP17=ON \
      -DCMAKE_INSTALL_PREFIX=/usr/local

make -j$(nproc)
sudo make install
```

### Frame ID Naming convention:

> The `system_info.yaml` file is structured as follows.

```yaml
radars:
  frame_id: identifier  
```

> If you want to display an identifier instead of frame_id, enter frame_id: identifier in the `system_info.yaml` file.

```yaml
radars:
  27c06058: FRONT RIGHT
  0b089dfa: REAR LEFT
```

**Compile**:

```bash
$ cd ~/share/ros2_ws
$ colcon build --cmake-args -DCMAKE_BUILD_TYPE=Relaese --packages-up-to au_4d_radar
```

**Running Radar Node using launch actions**:

```bash
$ cd ~/share/ros2_ws
source install/local_setup.bash
ros2 launch au_4d_radar run_radar.launch.py
```

### Run-time composition using ROS services

**Run in shell 1**:

> In the first shell, start the component container:

```bash
$ cd ~/share/ros2_ws
source install/local_setup.bash
ros2 run rclcpp_components component_container
```

**Run in shell 2**:

> In the second shell load device_au_radar_node node

```bash
$ cd ~/share/ros2_ws
source install/local_setup.bash
ros2 component load /ComponentManager au_4d_radar au_4d_radar::device_au_radar_node
```

**Run in shell 3**:

> In the third shell load Listener node(If you want to check subscriptions for messages from the device_au_radar_node publisher)

```bash
$ cd ~/share/ros2_ws
source install/local_setup.bash
ros2 component load /ComponentManager au_4d_radar au_4d_radar::Listener
```
