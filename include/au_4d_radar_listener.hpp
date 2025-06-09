
#ifndef AU4DRADAR__LISTENER_COMPONENT_HPP
#define AU4DRADAR__LISTENER_COMPONENT_HPP

#include "rclcpp/rclcpp.hpp"
#include <radar_msgs/msg/radar_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace au_4d_radar_listener
{

class listener_au_radar_node : public rclcpp::Node
{
public:
  explicit listener_au_radar_node(const rclcpp::NodeOptions & options);

private:
  rclcpp::Subscription<radar_msgs::msg::RadarScan>::SharedPtr sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub2_;  
  rclcpp::Time last_msg_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace au_4d_radar_listener

#endif  // AU4DRADAR__LISTENER_COMPONENT_HPP
