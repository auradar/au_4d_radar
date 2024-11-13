#ifndef __ADM_TF_LISTENER_NODE_HPP__
#define __ADM_TF_LISTENER_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

namespace au_4d_radar
{
class device_au_radar_node;

class AdmTFListener {

public:
    AdmTFListener(device_au_radar_node* node);
    ~AdmTFListener();

private:
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::TimerBase::SharedPtr timer_;

    void lookupTransform();
    std::tuple<double, double, double> TransformToRPY(const geometry_msgs::msg::TransformStamped& transform);

    device_au_radar_node* radar_node_;
};

}  // namespace au_4d_radar
#endif // __ADM_TF_LISTENER_NODE_HPP__
