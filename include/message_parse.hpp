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

        void parse_radar_data(uint8_t *p_buff, uint32_t *message_type, radar_msgs::msg::RadarScan &radar_scan_msg, radar_msgs::msg::RadarTracks &radar_tracks_msg);

    private:
        uint32_t sequence_id_;
        std::string frame_id_;
        uint32_t stamp_tv_sec_;
        uint32_t stamp_tv_nsec_;
    };
}

#endif // MESSAGE_PARSE_HPP
