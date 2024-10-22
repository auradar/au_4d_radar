"""Launch a AuRadar4D and a listener in a component container."""

import launch
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='radar_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='au_4d_radar',
                    plugin='au_4d_radar::device_au_radar_node',
                    name='device_au_radar_node')#,
                # ComposableNode(
                #     package='au_4d_radar',
                #     plugin='au_4d_radar::Listener',
                #     name='listener')
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])
