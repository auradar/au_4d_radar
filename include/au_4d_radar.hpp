#ifndef AU_4D_RADAR_HPP
#define AU_4D_RADAR_HPP

#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "mon_msgs/msg/radar_health.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "heart_beat.hpp"
#include "radar_packet_handler.hpp"
#include "adm_tf_listener.hpp"

namespace au_4d_radar
{
    const std::string DEFAULT_IP = "255.255.255.255" ;    // "255.255.255.255" "192.168.10.238"

    class device_au_radar_node: public rclcpp::Node
    {
    public:
        explicit device_au_radar_node(const rclcpp::NodeOptions& options);
        void publishHeartbeat(mon_msgs::msg::RadarHealth& radar_health_msg);
        void publishRadarPointCloud2(sensor_msgs::msg::PointCloud2& radar_cloud_msg);
        void publishRadarScanMsg(radar_msgs::msg::RadarScan &radar_scan_msg);
        void publishRadarTrackMsg(radar_msgs::msg::RadarTracks &radar_tracks_msg);

        Heartbeat heart_beat_;
        RadarPacketHandler radar_handler_;
        AdmTFListener adm_tf_listener_;

    private:
        template<typename Param>
        void get_param(rclcpp::Node::SharedPtr nh, const std::string& name, Param& variable);

        int initInterruptHandler(void);
        static void interruptHandler(int sig);
        void publish();

        rclcpp::Publisher<radar_msgs::msg::RadarScan>::SharedPtr pub_radar_scan;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_radar_point_cloud2;
        rclcpp::Publisher<radar_msgs::msg::RadarTracks>::SharedPtr pub_radar_track;
        rclcpp::Publisher<mon_msgs::msg::RadarHealth>::SharedPtr pub_radar_mon;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr timer_mon_;
        std::mutex mtx_msg_publisher;
        static uint32_t temp_cnt;

        static device_au_radar_node* instance_;
    };

} // namespace au_4d_radar

#endif  // AU_4D_RADAR_HPP

