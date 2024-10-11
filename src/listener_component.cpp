
#include "listener_component.hpp"

#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <radar_msgs/msg/radar_scan.hpp>

namespace au_4d_radar
{

Listener::Listener(const rclcpp::NodeOptions & options)
: Node("listener", options)
{
  auto callback =
    [this](radar_msgs::msg::RadarScan msg) -> void
    {
      RCLCPP_INFO(this->get_logger(), "frame_id: [%s]", msg.header.frame_id.c_str());
      std::flush(std::cout);
    };

  sub_ = create_subscription<radar_msgs::msg::RadarScan>("/device/au/radar/scan", rclcpp::SensorDataQoS() , callback);
}

}  // namespace au_4d_radar

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(au_4d_radar::Listener)
