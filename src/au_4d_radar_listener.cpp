
/**
 * @file au_4d_radar_listener.cpp
 * @author Antonio Ko(antonioko@au-sensor.com)
 * @brief radar_msgs listener_au_radar_node Processing
 * @version 1.1
 * @date 2025-06-09
 *
 * @copyright Copyright AU (c) 2025
 *
 */

#include "au_4d_radar_listener.hpp"

#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
// #include <radar_msgs/msg/radar_scan.hpp>

namespace au_4d_radar_listener
{

listener_au_radar_node::listener_au_radar_node(const rclcpp::NodeOptions & options)
: Node("listener_au_radar_node", options)
{
#ifdef DEBUG_BUILD
  if (rcutils_logging_set_logger_level(this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG) != RCUTILS_RET_OK) {
      RCLCPP_WARN(this->get_logger(), "Failed to set logger level to DEBUG");
  }
#endif

#if 1
  auto callback =
    [this](radar_msgs::msg::RadarScan msg) -> void
    {
      rclcpp::Time now = msg.header.stamp;
      double period = (now - last_msg_time_).seconds();
      double hz = 1.0 / period;
      last_msg_time_ = now;

      uint32_t time_sync_cloud = msg.header.stamp.nanosec / 10000000;
      RCLCPP_DEBUG(this->get_logger(), "id: %s %02u ms period: %.4fs (%.2f Hz)", msg.header.frame_id.c_str(), time_sync_cloud, period, hz);
      std::flush(std::cout);
    };

  sub_ = create_subscription<radar_msgs::msg::RadarScan>("/device/au/radar/scan", rclcpp::SensorDataQoS() , callback);
#endif


#if 0
  auto callback_point_cloud2 =
    [this](sensor_msgs::msg::PointCloud2 msg) -> void
    {
      rclcpp::Time now = msg.header.stamp;
      double period = (now - last_msg_time_).seconds();
      double hz = 1.0 / period;
      last_msg_time_ = now;

      uint32_t time_sync_cloud = msg.header.stamp.nanosec / 10000000;
      RCLCPP_DEBUG(this->get_logger(), "id: %s %02u ms period: %.4fs (%.2f Hz)", msg.header.frame_id.c_str(), time_sync_cloud, period, hz);
      std::flush(std::cout);
    };

  sub2_ = create_subscription<sensor_msgs::msg::PointCloud2>("/device/au/radar/point_cloud2", rclcpp::SensorDataQoS() , callback_point_cloud2);
#endif

}

}  // namespace au_4d_radar_listener

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(au_4d_radar_listener::listener_au_radar_node)
