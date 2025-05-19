/**
 * @file adm_tf_listener.cpp
 * @author Antonio Ko(antonioko@au-sensor.com)
 * @brief TF Listener Processing
 * @version 1.1
 * @date 2025-5-19
 *
 * @copyright Copyright AU (c) 2025
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

    RCLCPP_DEBUG(radar_node_->get_logger(), "AdmTFListener created!");
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

        std::vector<std::string> available_frames = tf_buffer_->getAllFrameNames();

        for (const auto & radar : radar_links) {
            if (std::find(available_frames.begin(), available_frames.end(), radar) == available_frames.end()) {
                // RCLCPP_DEBUG(radar_node_->get_logger(), "Radar frame %s not available - skipping", radar.c_str());
                continue;
            }

            if (std::find(available_frames.begin(), available_frames.end(), "base_link") == available_frames.end()) {
                RCLCPP_WARN(radar_node_->get_logger(), "base_link frame not available");
                break;
            }

            transform = tf_buffer_->lookupTransform("base_link", radar, tf2::TimePointZero);
            auto [roll, pitch, yaw] = TransformToRPY(transform);

            RadarInfo current_info = YamlParser::getRadarInfo(radar);
            RadarInfo new_info = current_info;

            new_info.x = transform.transform.translation.x;
            new_info.y = transform.transform.translation.y;
            new_info.z = transform.transform.translation.z;
            new_info.roll = roll;
            new_info.pitch = pitch;
            new_info.yaw = yaw;

            if (new_info.x != current_info.x ||
                new_info.y != current_info.y ||
                new_info.z != current_info.z ||
                new_info.roll != current_info.roll ||
                new_info.pitch != current_info.pitch ||
                new_info.yaw != current_info.yaw) {

                YamlParser::setRadarInfo(radar, new_info);

                RCLCPP_DEBUG(radar_node_->get_logger(), "Frame_id %s: translation (x: %f, y: %f, z: %f)",
                            new_info.frame_id.c_str(), new_info.x, new_info.y, new_info.z);
                RCLCPP_DEBUG(radar_node_->get_logger(), "Transform rotation (roll: %lf, pitch: %lf, yaw: %lf)",
                            new_info.roll, new_info.pitch, new_info.yaw);
            }
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

  //  RCLCPP_DEBUG(radar_node_->get_logger(), "child_frame_id: %s translation (qx: %f, qy: %f, qz: %f, qw: %f)",
  //              transform.child_frame_id.c_str(), qx, qy, qz, qw);

    // Convert quaternion to roll, pitch, yaw
    double roll, pitch, yaw;
    tf2::Matrix3x3 m(quaternion);
    m.getRPY(roll, pitch, yaw);

    return std::make_tuple(roll, pitch, yaw);
}


} // namespace au_4d_radar