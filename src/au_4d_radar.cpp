  /**
 * @file au_4d_radar.cpp
 * @author antonioko@au-sensor.com
 * @brief
 * @version 1.1
 * @date 2024-09-11
 *
 * @copyright Copyright AU (c) 2024
 *
 */

#include "au_4d_radar.hpp"

#define PUB_TIME 	10ms

namespace au_4d_radar {

device_au_radar_node* device_au_radar_node::instance_ = nullptr;

device_au_radar_node::device_au_radar_node(const rclcpp::NodeOptions & options)
: Node("device_au_radar_node", options),
  heart_beat_(this), radar_handler_(this)//, message_parser_(this)
{
    instance_ = this;
    pub_radar_scan = this->create_publisher<radar_msgs::msg::RadarScan>(
                    "/device/au/radar/scan",
                    rclcpp::SensorDataQoS());

    pub_radar_track = this->create_publisher<radar_msgs::msg::RadarTracks>(
                    "/device/au/radar/track",
                    rclcpp::SensorDataQoS());

    pub_radar_mon = this->create_publisher<mon_msgs::msg::RadarHealth>(
                    "/device/au/radar/status",
                    rclcpp::SensorDataQoS());

    // https://docs.ros2.org/foxy/api/rmw/types_8h.html
    rclcpp::QoS qos = rclcpp::SensorDataQoS();
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE); // RMW_QOS_POLICY_RELIABILITY_RELIABLE RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
    // pub_radar_point_cloud2 = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    //                 "/device/au/radar/point_cloud2", qos);
    pub_radar_front = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                    "/device/au/radar_front", qos);
    pub_radar_front_right = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                    "/device/au/radar_front_right", qos);
    pub_radar_rear_right = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                    "/device/au/radar_rear_right", qos);
    pub_radar_rear_left = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                    "/device/au/radar_rear_left", qos);
    pub_radar_front_left = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                    "/device/au/radar_front_left", qos);

    initInterruptHandler();
    YamlParser::init();
    heart_beat_.start();
    radar_handler_.start();

    RCLCPP_INFO(rclcpp::get_logger("radar_node"), "Start AU 4D Radar Driver Node");
}

void device_au_radar_node::interruptHandler(int sig) {
    if (instance_ != nullptr) {
        RCLCPP_ERROR(rclcpp::get_logger("interruptHandler"), "signum=%d", sig);
        if (sig == SIGINT || sig == SIGHUP || sig == SIGKILL || sig == SIGSEGV || sig == SIGTERM) {
            RCLCPP_ERROR(rclcpp::get_logger("radar_node"), "interruptHandler performed");

            instance_->heart_beat_.stop();
            instance_->radar_handler_.stop();

            exit(0);
        }
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("interruptHandler"), "Instance is null. Ignoring signal.");
    }
}

template<typename Param>
void device_au_radar_node::get_param(rclcpp::Node::SharedPtr nh, const std::string& name, Param& variable) {
    using var_type = std::remove_reference_t<decltype(variable)>;

    if (!nh->has_parameter(name)) {
        variable = nh->declare_parameter<var_type>(name, variable);
    } else {
        nh->get_parameter(name, variable);
    }
}

void device_au_radar_node::publishRadarScanMsg(radar_msgs::msg::RadarScan &radar_scan_msg) {
    std::lock_guard<std::mutex> lock(mtx_msg_publisher);
    if (radar_scan_msg.header.frame_id.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("publishRadarScanMsg"), "Empty frame ID in Scan message");
        return;
    }
    pub_radar_scan->publish(radar_scan_msg);
    // RCLCPP_INFO(rclcpp::get_logger("radar_node"), "pub_radar_scan id %s 50ms %02u",
    //     radar_scan_msg.header.frame_id.c_str(), radar_scan_msg.header.stamp.nanosec / 10000000);
}

void device_au_radar_node::publishRadarPointCloud2(sensor_msgs::msg::PointCloud2& radar_cloud_msg) {
    std::lock_guard<std::mutex> lock(mtx_msg_publisher);
    if (radar_cloud_msg.header.frame_id.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("publishRadarPointCloud2"), "Empty frame ID in PointCloud2 message");
        return;
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher = nullptr;
    if (radar_cloud_msg.header.frame_id == "RADAR_FRONT") {
        publisher = pub_radar_front;
    } else if (radar_cloud_msg.header.frame_id == "RADAR_FRONT_RIGHT") {
        publisher = pub_radar_front_right;
    } else if (radar_cloud_msg.header.frame_id == "RADAR_REAR_RIGHT") {
        publisher = pub_radar_rear_right;
    } else if (radar_cloud_msg.header.frame_id == "RADAR_REAR_LEFT") {
        publisher = pub_radar_rear_left;
    } else if (radar_cloud_msg.header.frame_id == "RADAR_FRONT_LEFT") {
        publisher = pub_radar_front_left;
    } else {
        RCLCPP_WARN(rclcpp::get_logger("publishRadarPointCloud2"), "Unknown frame ID: %s", radar_cloud_msg.header.frame_id.c_str());
        return;
    }

    if (publisher) {
        publisher->publish(radar_cloud_msg);
        RCLCPP_INFO(rclcpp::get_logger("radar_node"), "pub_radar_point_cloud2 id %s 50ms %02u",
           radar_cloud_msg.header.frame_id.c_str(), radar_cloud_msg.header.stamp.nanosec / 10000000);
    }
}

void device_au_radar_node::publishRadarTrackMsg(radar_msgs::msg::RadarTracks &radar_tracks_msg) {
    std::lock_guard<std::mutex> lock(mtx_msg_publisher);
    pub_radar_track->publish(radar_tracks_msg);
}

void device_au_radar_node::publishHeartbeat(mon_msgs::msg::RadarHealth& radar_health_msg) {
    std::lock_guard<std::mutex> lock(mtx_msg_publisher);
    // RCLCPP_INFO(rclcpp::get_logger("radar_node"),
    // "pub_radar_mon hostname : %s status: %u tv_sec: %u",
    // radar_health_msg.client_hostname.c_str(), radar_health_msg.status, radar_health_msg.tv_sec);
    pub_radar_mon->publish(radar_health_msg);
}

int device_au_radar_node::initInterruptHandler(void) {
    signal(SIGINT, interruptHandler);
    signal(SIGHUP, interruptHandler);
    signal(SIGSEGV, interruptHandler);
    // signal(SIGKILL, interruptHandler);
    // signal(SIGTERM, interruptHandler);
    return 0;
}

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(au_4d_radar::device_au_radar_node)
