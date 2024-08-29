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

#include "config.hpp"


namespace au_4d_radar
{

#define HEADER_SCAN 			0x5343414e 
#define HEADER_TRACK 			0x54524143
#define HEADER_MON 				0x4d4f4e49

    struct tsPacketHeader
    {
        uint32_t ui32SB;
        uint32_t uniq_id;             
        uint32_t tv_sec;
        uint32_t tv_nsec;
        uint32_t ui32FN;
        float    f32CT;
        uint32_t ui32TPN;
        uint32_t ui32PN;
        uint16_t ui16TPCKN;
        uint16_t ui16PCKN;
    };

    class MessageParser
    {
    public:
        MessageParser()  = default;
        ~MessageParser() = default;  

        void makeRadarPointCloud2Mssg(uint8_t *p_buff, sensor_msgs::msg::PointCloud2& cloud_msg);       
        void makeRadarScanMssg(uint8_t *p_buff, radar_msgs::msg::RadarScan& radar_scan_msg);
        void makeRadarTracksMssg(uint8_t *p_buff, radar_msgs::msg::RadarTracks& radar_tracks_msg);
        void parseRadarData(uint8_t *p_buff, uint32_t *message_type, 
            #if (POINT_CLOUD2)
            sensor_msgs::msg::PointCloud2& radar_cloud_msg,
            #else
            radar_msgs::msg::RadarScan& radar_scan_msg,
            #endif
            radar_msgs::msg::RadarTracks& radar_tracks_msg);

    private:
        uint32_t sequence_id_;
        std::string frame_id_;
        uint32_t stamp_tv_sec_;
        uint32_t stamp_tv_nsec_;
    };
}

#endif // MESSAGE_PARSE_HPP
