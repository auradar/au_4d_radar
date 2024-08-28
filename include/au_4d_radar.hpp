  /**
 * @file au_4d_radar.hpp
 * @author kisoo.kim@au-sensor.com, antonioko@au-sensor.com
 * @version 0.1
 * @date 2024-04-05
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef AU_4D_RADAR_HPP
#define AU_4D_RADAR_HPP

#include "rclcpp/rclcpp.hpp"
#include "mon_msgs/msg/radar_health.hpp"


#include "radar_packet_handler.hpp"  
#include "message_parse.hpp"
#include "heart_beat.hpp" 


namespace au_4d_radar
{

    const std::string DEFAULT_IP = "255.255.255.255" ;    // "255.255.255.255" "192.168.10.238"
   
    class device_au_radar_node: public rclcpp::Node
    {
    public:
        explicit device_au_radar_node(const rclcpp::NodeOptions & options);
        void publishHeartbeat(mon_msgs::msg::RadarHealth &radar_health_msg);
        void publishRadarData(uint32_t message_type, radar_msgs::msg::RadarScan &radar_scan_msg, radar_msgs::msg::RadarTracks &radar_tracks_msg);

        Heartbeat heart_beat_; 
        RadarPacketHandler radar_handler_;    
        
    private:
        template<typename Param>
        void get_param(rclcpp::Node::SharedPtr nh, const std::string& name, Param& variable);

        int initInterruptHandler(void);
        int initRadarPacketHandler(void);
        int initHeartbeatHandler(void);
        void stopRadarPacketHandler(void);
        void stopHeartbeatHandler(void);
        static void interruptHandler(int sig);
        void publish();
        void monitor();

        rclcpp::Publisher<radar_msgs::msg::RadarScan>::SharedPtr pub_radar_scan;
        rclcpp::Publisher<radar_msgs::msg::RadarTracks>::SharedPtr pub_radar_track;
        rclcpp::Publisher<mon_msgs::msg::RadarHealth>::SharedPtr pub_radar_mon;

        radar_msgs::msg::RadarScan radar_scan_msg;
        radar_msgs::msg::RadarTracks radar_tracks_msg;
        mon_msgs::msg::RadarHealth radar_health_msg;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr timer_mon_;

        static uint32_t temp_cnt;

        MessageParser message_parser_; 
        
        static device_au_radar_node* instance_;
    };
}

#endif  // AU_4D_RADAR_HPP

