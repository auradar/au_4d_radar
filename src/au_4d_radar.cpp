  /**
 * @file au_4d_radar.cpp
 * @author kisoo.kim@au-sensor.com, antonioko@au-sensor.com
 * @brief 
 * @version 1.0
 * @date 2024-08-23
 * 
 * @copyright Copyright AU (c) 2024
 * 
 */

#include "au_4d_radar.hpp"

#define PUB_TIME 	10ms
#define MON_TIME 	1000ms
#define UDP_MTU		1500

using namespace std::chrono_literals;

namespace au_4d_radar {

device_au_radar_node* device_au_radar_node::instance_ = nullptr;

device_au_radar_node::device_au_radar_node(const rclcpp::NodeOptions & options)
: Node("device_au_radar_node", options), 
  heart_beat_(this), radar_handler_(this)  
{
    instance_ = this;
  
    // timer_ = this->create_wall_timer(
    //     PUB_TIME, 
    //     std::bind(&device_au_radar_node::publish, this));

    rclcpp::QoS qos = rclcpp::SensorDataQoS();
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    pub_radar_point_cloud2 = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                    "/device/au/radar/point_cloud2", qos);

    // pub_radar_point_cloud2 = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    //                 "/device/au/radar/point_cloud2", rclcpp::SensorDataQoS());                    

    pub_radar_scan = this->create_publisher<radar_msgs::msg::RadarScan>(
                    "/device/au/radar/scan",
                    rclcpp::SensorDataQoS());

    pub_radar_track = this->create_publisher<radar_msgs::msg::RadarTracks>(
                    "/device/au/radar/track",
                    rclcpp::SensorDataQoS());

    pub_radar_mon = this->create_publisher<mon_msgs::msg::RadarHealth>(
                      "/device/au/radar/status",
                      rclcpp::SensorDataQoS());

    initInterruptHandler();
    initHeartbeatHandler();
    initRadarPacketHandler();

    RCLCPP_INFO(rclcpp::get_logger("device_au_radar_node"), "Start AU 4D Radar Driver Node");
}

void device_au_radar_node::interruptHandler(int sig) {
    RCLCPP_ERROR(rclcpp::get_logger("interruptHandler"), "signum=%d", sig);

    if (sig == SIGINT || sig == SIGHUP || sig == SIGKILL || sig == SIGSEGV || sig == SIGTERM) {
        RCLCPP_INFO(rclcpp::get_logger("device_au_radar_node"), "device_au_radar_node::interruptHandler performed");

        instance_->stopRadarPacketHandler();
        instance_->stopHeartbeatHandler();

        exit(0);
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

void device_au_radar_node::publishRadarData(uint32_t message_type, radar_msgs::msg::RadarScan &radar_scan_msg, radar_msgs::msg::RadarTracks &radar_tracks_msg) {
    if(message_type == HEADER_SCAN){
        pub_radar_scan->publish(radar_scan_msg);
    } else if(message_type == HEADER_TRACK){
        pub_radar_track->publish(radar_tracks_msg);  
    }
}

void device_au_radar_node::publishRadarPointCloud2(uint32_t message_type, sensor_msgs::msg::PointCloud2& radar_cloud_msg, radar_msgs::msg::RadarTracks& radar_tracks_msg) {
    if(message_type == HEADER_SCAN){
        pub_radar_point_cloud2->publish(radar_cloud_msg);
    } else if(message_type == HEADER_TRACK){
        pub_radar_track->publish(radar_tracks_msg);  
    }
}

void device_au_radar_node::publishHeartbeat(mon_msgs::msg::RadarHealth& radar_health_msg) {
    // RCLCPP_INFO(rclcpp::get_logger("device_au_radar_node"), 
    // "publish radar health msgs client_hostname : %s status: %u tv_sec: %u", 
    // radar_health_msg.client_hostname.c_str(), radar_health_msg.status, radar_health_msg.tv_sec);  

    pub_radar_mon->publish(radar_health_msg);  
}

int device_au_radar_node::initInterruptHandler(void) {
    signal(SIGINT, interruptHandler);
    signal(SIGHUP, interruptHandler);
    signal(SIGKILL, interruptHandler);
    signal(SIGSEGV, interruptHandler);
    signal(SIGTERM, interruptHandler);
    return 0;
}

int device_au_radar_node::initHeartbeatHandler(void) {
    heart_beat_.start();
    return 0;
}

void device_au_radar_node::stopHeartbeatHandler(void) {
    heart_beat_.stop();
}

int device_au_radar_node::initRadarPacketHandler(void) {
    radar_handler_.start();
    return 0;
}

void device_au_radar_node::stopRadarPacketHandler(void) {
    radar_handler_.stop();
}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
RCLCPP_COMPONENTS_REGISTER_NODE(au_4d_radar::device_au_radar_node)
