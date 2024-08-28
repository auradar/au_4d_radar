  /**
 * @file message_parse.cpp
 * @author kisoo.kim@au-sensor.com, antonioko@au-sensor.com
 * @brief Implementation of the MessageParser class for parsing radar data.
 * @version 1.0
 * @date 2024-08-23
 * 
 * @copyright Copyright AU (c) 2024
 * 
 */


#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "message_parse.hpp"
#include "uuid_helper.hpp"
#include "util/conversion.hpp"

namespace au_4d_radar
{

void MessageParser::parse_radar_data(uint8_t *p_buff, uint32_t *message_type, radar_msgs::msg::RadarScan &radar_scan_msg, radar_msgs::msg::RadarTracks &radar_tracks_msg)
{
    uint32_t idx = 0;
    tsPacketHeader header = {};
    std::stringstream ss;

    uint32_t id = Conversion::little_endian_to_uint32(&p_buff[0]);
    *message_type = id;

    if(id == HEADER_SCAN) {
        idx += 4;
        header.uniq_id = Conversion::little_endian_to_uint32(&p_buff[idx]);
        idx += 4;
        header.tv_sec = Conversion::little_endian_to_uint32(&p_buff[idx]);        
        idx += 4;
        header.tv_nsec = Conversion::little_endian_to_uint32(&p_buff[idx]);
        idx += 4;
        header.ui32FN = Conversion::little_endian_to_uint32(&p_buff[idx]);
        idx += 4;
        header.f32CT = Conversion::convert_to_float(&p_buff[idx]);
        idx += 4;
        header.ui32TPN = Conversion::little_endian_to_uint32(&p_buff[idx]);
        idx += 4;
        header.ui32PN = Conversion::little_endian_to_uint32(&p_buff[idx]);
        idx += 4;
        header.ui16TPCKN = Conversion::little_endian_to_uint16(&p_buff[idx]);
        idx += 2;
        header.ui16PCKN = Conversion::little_endian_to_uint16(&p_buff[idx]);
        idx += 2;

        if(header.ui32PN > 60){ // 60
            RCLCPP_ERROR(rclcpp::get_logger("MessageParser"), "Failed to decode parse_radar_data ui32PN: %u", header.ui32PN);               
            return;
        }

        // https://github.com/ros2/common_interfaces/blob/rolling/std_msgs/msg/Header.msg
        //sequence_id_ = header.ui32FN; 
        ss << std::hex << header.uniq_id;
        frame_id_ = ss.str();
        stamp_tv_sec_ = header.tv_sec;
        stamp_tv_nsec_ = header.tv_nsec;

        radar_scan_msg.header.frame_id = frame_id_;
        radar_scan_msg.header.stamp.sec = stamp_tv_sec_;
        radar_scan_msg.header.stamp.nanosec = stamp_tv_nsec_;

		//Point cloud
        for(uint32_t i = 0; i < header.ui32PN; i++)
        {
            radar_msgs::msg::RadarReturn return_msg;
            return_msg.range = Conversion::convert_to_float(&p_buff[idx]);
            idx += 4;
            return_msg.doppler_velocity = Conversion::convert_to_float(&p_buff[idx]);
            idx += 4;		    
            return_msg.azimuth = Conversion::convert_to_float(&p_buff[idx]);
            idx += 4;
            return_msg.elevation = Conversion::convert_to_float(&p_buff[idx]);
            idx += 4;
            return_msg.amplitude = Conversion::convert_to_float(&p_buff[idx]);
            idx += 4;

            radar_scan_msg.returns.push_back(return_msg);
        }
                
    } else if(id == HEADER_TRACK) {
        radar_tracks_msg.header.frame_id = frame_id_;	
        radar_tracks_msg.header.stamp.sec = stamp_tv_sec_;	
        radar_tracks_msg.header.stamp.nanosec = stamp_tv_nsec_;

		//Tracking
        for(int i = 0; i < 3; i++)
        {
            radar_msgs::msg::RadarTrack radar_data_msg;
            radar_data_msg.uuid = tier4_autoware_utils::generateUUID();
            radar_data_msg.position.x = 1.0;
            radar_data_msg.position.y = i;
            radar_data_msg.position.z = i;

            radar_data_msg.velocity.x = 1.0;
            radar_data_msg.velocity.y = 2.0;
            radar_data_msg.velocity.z = 3.0;

            radar_data_msg.acceleration.x = 1.0;
            radar_data_msg.acceleration.y = 2.0;
            radar_data_msg.acceleration.z = 3.0;

            radar_data_msg.size.x = 1.0;
            radar_data_msg.size.y = 2.0;
            radar_data_msg.size.z = 3.0;

            radar_data_msg.classification = 1;			
            radar_data_msg.position_covariance[0] = 1.0;

            radar_tracks_msg.tracks.push_back(radar_data_msg);
        }
      
    } else if(id == HEADER_MON) {
        RCLCPP_INFO(rclcpp::get_logger("MessageParser"), "HEADER_MON message");         
    }  else {
        RCLCPP_INFO(rclcpp::get_logger("MessageParser"), "Failed to decode message id: %08x", id);           
    }

    if(id == HEADER_SCAN || id == HEADER_TRACK){
            RCLCPP_INFO(rclcpp::get_logger("MessageParser"), 
            "id: %08x frame_id: %s ui32FN: %u ui32TPN: %u ui32PN: %u", 
             id, frame_id_.c_str(), header.ui32FN, header.ui32TPN, header.ui32PN);                  
    }
}


}
