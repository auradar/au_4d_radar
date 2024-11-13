/**
 * @file adm_tf_listener.cpp
 * @author Antonio Ko(antonioko@au-sensor.com)
 * @brief TF Listener Processing
 * @version 1.0
 * @date 2024-10-29
 *
 * @copyright Copyright AU (c) 2024
 *
 */

#include <memory>
#include <string>
#include "tf2_ros/create_timer_ros.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "rclcpp/rclcpp.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "au_4d_radar.hpp"
#include "util/yamlParser.hpp"

namespace au_4d_radar {

AdmTFListener::AdmTFListener(device_au_radar_node* node): radar_node_(node) {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(radar_node_->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(radar_node_->get_node_base_interface(), radar_node_->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    timer_ = radar_node_->create_wall_timer(std::chrono::seconds(1), std::bind(&AdmTFListener::lookupTransform, this));

    RCLCPP_INFO(radar_node_->get_logger(), "AdmTFListener created!");
}

void AdmTFListener::lookupTransform() {
    geometry_msgs::msg::TransformStamped transform;

    try {
        std::vector<std::string> radar_links = {
            "RADAR_FRONT",
            "RADAR_FRONT_RIGHT",
            "RADAR_FRONT_LEFT",
            "RADAR_REAR_RIGHT",
            "RADAR_REAR_LEFT"
        };

        for (const auto & radar : radar_links) {
            transform = tf_buffer_->lookupTransform("base_link", radar, tf2::TimePointZero);
            auto [roll, pitch, yaw] = TransformToRPY(transform);

            RadarInfo radar_info = YamlParser::getRadarInfo(radar);
            radar_info.x = transform.transform.translation.x;
            radar_info.y = transform.transform.translation.y;
            radar_info.z = transform.transform.translation.z;
            radar_info.roll = roll;
            radar_info.pitch = pitch;
            radar_info.yaw = yaw;
            YamlParser::setRadarInfo(radar, radar_info);

  //          RCLCPP_INFO(radar_node_->get_logger(), "Frame_id %s: translation (x: %f, y: %f, z: %f)",
  //                      radar_info.frame_id.c_str(), radar_info.x, radar_info.y, radar_info.z);
   //         RCLCPP_INFO(radar_node_->get_logger(), "Transform rotation (roll: %lf, pitch: %lf, yaw: %lf)",
  //                      radar_info.roll, radar_info.pitch, radar_info.yaw);
        }
    }
    catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(radar_node_->get_logger(), "Could not transform: %s", ex.what());
    }
}

std::tuple<double, double, double> AdmTFListener::TransformToRPY(const geometry_msgs::msg::TransformStamped& transform) {
    double qx = transform.transform.rotation.x;
    double qy = transform.transform.rotation.y;
    double qz = transform.transform.rotation.z;
    double qw = transform.transform.rotation.w;

    tf2::Quaternion quaternion(qx, qy, qz, qw);

  //  RCLCPP_INFO(radar_node_->get_logger(), "child_frame_id: %s translation (qx: %f, qy: %f, qz: %f, qw: %f)",
  //              transform.child_frame_id.c_str(), qx, qy, qz, qw);

    // Convert quaternion to roll, pitch, yaw
    double roll, pitch, yaw;
    tf2::Matrix3x3 m(quaternion);
    m.getRPY(roll, pitch, yaw);

    return std::make_tuple(roll, pitch, yaw);
}


} // namespace au_4d_radar