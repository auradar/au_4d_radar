
/**
 * @file listener_node.cpp
 * @author Antonio Ko(antonioko@au-sensor.com)
 * @brief radar_msgs Listener Processing
 * @version 1.0
 * @date 2025-5-19
 *
 * @copyright Copyright AU (c) 2025
 *
 */

#include "listener_node.hpp"

#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <radar_msgs/msg/radar_scan.hpp>

namespace au_4d_radar
{

Listener::Listener(const rclcpp::NodeOptions & options)
: Node("listener_node", options)
{
#ifdef DEBUG_BUILD
  if (rcutils_logging_set_logger_level(this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG) != RCUTILS_RET_OK) {
      RCLCPP_WARN(this->get_logger(), "Failed to set logger level to DEBUG");
  }
#endif
  
  auto callback =
    [this](radar_msgs::msg::RadarScan msg) -> void
    {
      RCLCPP_DEBUG(this->get_logger(), "frame_id: [%s]", msg.header.frame_id.c_str());
      std::flush(std::cout);
    };

  sub_ = create_subscription<radar_msgs::msg::RadarScan>("/device/au/radar/scan", rclcpp::SensorDataQoS() , callback);
}

}  // namespace au_4d_radar

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(au_4d_radar::Listener)
