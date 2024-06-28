cmake_minimum_required(VERSION 3.8)

project(au_4d_radar)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(radar_msgs REQUIRED)
find_package(mon_msgs REQUIRED)

include_directories(include)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

add_library(device_au_radar_node SHARED
  src/au_4d_radar.cpp
  src/cm.cpp
  src/socket.cpp)

target_compile_definitions(device_au_radar_node
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(device_au_radar_node
  "rclcpp"
  "rclcpp_components"  
  "sensor_msgs"  
  "nav_msgs"
  "radar_msgs"
  "mon_msgs")

rclcpp_components_register_nodes(device_au_radar_node "au_4d_radar::device_au_radar_node")
set(node_plugins "${node_plugins}au_4d_radar::device_au_radar_node;$<TARGET_FILE:device_au_radar_node>\n")

# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

install(TARGETS
  device_au_radar_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()