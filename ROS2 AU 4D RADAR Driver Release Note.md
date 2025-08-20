## v1.0
* Support 1:1 communication between RADAR Sensor and ROS2 RADAR Component
* 1st Field Integration Completed


## v2.0
* Support N:1 communication between RADAR Sensor and ROS2 RADAR Component
* Support auto Connection between RADAR Sensors and RADAR Component
* Support ros2 publish for 1 packet unit
* Support Frame ID Naming convention
* Support POINT_CLOUD2 message publish
* 2nd Field Integration in K*City Completed


## v2.1
* Data processing by Frame ID (RADAR Sensor)
* Support ros2 publish in 1 Frame unit
* Development version (Field Integration not performed)


## v2.2
* Strengthened data race processing for data processing by Frame ID (RADAR Sensor) (multiple Tread Design)
* Added CRC to heartbeat message (Auto Connection between RADAR Sensors and RADAR Component)
* Development version (Field Integration not performed)


## v2.4
* Support for merging point cloud messages for multiple radars
* Radar Scan Message is published for each RADAR
* 3rd Field Integration in Seongwoo Motors Completed


## v3.0 (from v2.2)
* Each point cloud message for multiple radars *> TF Publisher (converted) *> Subscriber (RVIZ)
* Radar Point Cloud2 message is published for each RADAR
* support to assemble point cloud messages into a single message

## v10.0 (from v2.4)
* Transformed/merged point cloud messages for multiple radars *> Subscriber (RVIZ)
* Support transform using tf_listener
* support to assemble point cloud messages into a single message


## [R1] Release Note * <에이유> (2025-04-23)
> Description - R1 (from v10.0)
### Breaking Changes
* N/A

### New Features
* N/A

### Bug Fixes
* N/A

### Performance Improvements
* N/A

### Other Changes
* system_info.yaml 설정 파일에서 rpy 값을 radian 대신 degree로 변경 및 YAML Parser에 반영


## [R2] Release Note * <에이유> (2025-05-21)
> Description
### Breaking Changes
* FlatBuffers Version Change : v24.3.25 -> v25.2.10

### New Features
* 각 RADAR Sensor 네트워크 연결 상태 및 센서 내부 온도/에러 모니터링 추가

### Bug Fixes
* N/A

### Performance Improvements
* N/A

### Other Changes
* N/A

## [R3] Release Note * <에이유> (2025-06-19)
> Description
### Breaking Changes
* N/A

### New Features
* listener_au_radar_node point_cloud2 모니터링 추가

### Bug Fixes
* N/A

### Performance Improvements
* N/A

### Other Changes
* RCLCPP_DEBUG log 변경

## [R4] Release Note * <에이유> (2025-07-17)
> Description
>* Publish QOS POLICY 변경:    
> 모든 ROS2 Publish QOS POLICY 'best_effort'에서 'reliable'로 변경

### Breaking Changes
* N/A

### New Features
* N/A

### Bug Fixes
* N/A

### Performance Improvements
* N/A

### Other Changes
* N/A

## [R5] Release Note * <에이유> (2025-08-20)
> Description
>* Point Cloud 2 Message 변경:
> 카이스트 요청으로 Point Cloud 2 Message에 velocity 추가함.

### Breaking Changes
* N/A

### New Features
* N/A

### Bug Fixes
* N/A

### Performance Improvements
* N/A

### Other Changes
* N/A