  /**
 * @file message_parse.hpp
 * @author kisoo.kim@au-sensor.com, antonioko@au-sensor.com
 * @brief Implementation of the MessageParser class for parsing radar data.
 * @version 1.0
 * @date 2024-08-23
 * 
 * @copyright Copyright AU (c) 2024
 * 
 */

#ifndef MESSAGE_PARSE_HPP
#define MESSAGE_PARSE_HPP

#include <cstdint>
#include <string>
#include <boost/uuid/uuid.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <radar_msgs/msg/radar_return.hpp>
#include <radar_msgs/msg/radar_scan.hpp>
#include <radar_msgs/msg/radar_track.hpp>
#include <radar_msgs/msg/radar_tracks.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>


namespace au_4d_radar
{

    class device_au_radar_node;
    class MessageParser
    {
    public:
        MessageParser(device_au_radar_node* node);
        MessageParser()  = default;
        ~MessageParser() = default;

        void parsePointCloud2Msg(uint8_t *p_buff, sensor_msgs::msg::PointCloud2& radar_cloud_msg);
        void parseRadarScanMsg(uint8_t *p_buff, radar_msgs::msg::RadarScan& radar_scan_msg);
        void parseRadarTrackMsg(uint8_t *p_buff, radar_msgs::msg::RadarTracks& radar_tracks_msg);

    private:
        void makeRadarPointCloud2Msg(uint8_t *p_buff, sensor_msgs::msg::PointCloud2& cloud_msg); 
        void makeRadarScanMsg(uint8_t *p_buff, radar_msgs::msg::RadarScan& radar_scan_msg);
        void makeRadarTracksMsg(uint8_t *p_buff, radar_msgs::msg::RadarTracks& radar_tracks_msg);
        
        uint32_t sequence_id_;
        std::string frame_id_;
        uint32_t stamp_tv_sec_;
        uint32_t stamp_tv_nsec_;

        device_au_radar_node* radar_node_;
    };

}

#endif // MESSAGE_PARSE_HPP
