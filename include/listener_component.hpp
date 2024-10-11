
#ifndef AU4DRADAR__LISTENER_COMPONENT_HPP
#define AU4DRADAR__LISTENER_COMPONENT_HPP

//#include "composition/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include <radar_msgs/msg/radar_scan.hpp>

namespace au_4d_radar
{

class Listener : public rclcpp::Node
{
public:
  // COMPOSITION_PUBLIC
  explicit Listener(const rclcpp::NodeOptions & options);

private:
  rclcpp::Subscription<radar_msgs::msg::RadarScan>::SharedPtr sub_;
};

}  // namespace au_4d_radar

#endif  // AU4DRADAR__LISTENER_COMPONENT_HPP
