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
#include <vector>
#include <cmath>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "au_4d_radar.hpp"
#include "util/uuid_helper.hpp"
#include "util/conversion.hpp"
#include "util/util.hpp"

namespace au_4d_radar
{

void MessageParser::makeRadarPointCloud2Mssg(uint8_t *p_buff, sensor_msgs::msg::PointCloud2& cloud_msg) {
    uint32_t idx = 0;
    tsPacketHeader header = {};
    std::stringstream ss;
    const double deg2rad = 3.141592 / 180.0;    

    idx += 4;
    header.ui32UID = Conversion::littleEndianToUint32(&p_buff[idx]);
    idx += 4;
    header.ui32TS = Conversion::littleEndianToUint32(&p_buff[idx]);        
    idx += 4;
    header.ui32TN = Conversion::littleEndianToUint32(&p_buff[idx]);
    idx += 4;
    header.ui32FN = Conversion::littleEndianToUint32(&p_buff[idx]);
    idx += 4;
    header.f32CT = Conversion::convertToFloat(&p_buff[idx]);
    idx += 4;
    header.ui32TPN = Conversion::littleEndianToUint32(&p_buff[idx]);
    idx += 4;
    header.ui32PN = Conversion::littleEndianToUint32(&p_buff[idx]);
    idx += 4;
    header.ui16TPCKN = Conversion::littleEndianToUint16(&p_buff[idx]);
    idx += 2;
    header.ui16PCKN = Conversion::littleEndianToUint16(&p_buff[idx]);
    idx += 2;

    if(header.ui32PN > 60) { 
        RCLCPP_ERROR(rclcpp::get_logger("point_cloud2_msg"), "Failed to decode RadarPointCloud2Mssg ui32PN: %u", header.ui32PN);               
        return;
    }

    // https://github.com/ros2/common_interfaces/blob/rolling/std_msgs/msg/Header.msg
    //sequence_id_ = header.ui32FN; 
    ss << std::hex << header.ui32UID;
    frame_id_ = Util::readFrameIdFromYaml(ss.str());
    if(frame_id_.empty())
        frame_id_ = ss.str();

    stamp_tv_sec_ = header.ui32TS;
    stamp_tv_nsec_ = header.ui32TN;

    RCLCPP_INFO(rclcpp::get_logger("point_cloud2"), "UID %08x FN %u TPN %u PN %u TPCKN %u PCKN %u", 
                                                    header.ui32UID, header.ui32FN, header.ui32TPN, header.ui32PN, header.ui16TPCKN, header.ui16PCKN); 

    // https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/PointCloud2.msg
    cloud_msg.header.frame_id = frame_id_;
    cloud_msg.header.stamp.sec = stamp_tv_sec_;
    cloud_msg.header.stamp.nanosec = stamp_tv_nsec_;

    // Define PointCloud2 structure
    cloud_msg.height = 1;  // Unordered point cloud, height = 1
    cloud_msg.width = header.ui32PN;  // Number of points

    // Set fields for x, y, z, and intensity
    cloud_msg.fields.resize(4);

    cloud_msg.fields[0].name     = "x";
    cloud_msg.fields[0].offset   = 0;
    cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[0].count    = 1;

    cloud_msg.fields[1].name = "y";
    cloud_msg.fields[1].offset = 4;
    cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[1].count = 1;

    cloud_msg.fields[2].name = "z";
    cloud_msg.fields[2].offset = 8;
    cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[2].count = 1;

    cloud_msg.fields[3].name = "intensity";
    cloud_msg.fields[3].offset = 12;
    cloud_msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[3].count = 1;

    cloud_msg.is_bigendian = false;  
    cloud_msg.point_step = 16;  // Each point has 4 fields, each 4 bytes (float)
    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
    cloud_msg.is_dense = true;  // No invalid points

    // Resize data array to hold all points
    cloud_msg.data.resize(cloud_msg.row_step * cloud_msg.height);

    // Populate the point cloud data
    for (uint32_t i = 0; i < header.ui32PN; i++) {
        uint32_t index = Conversion::littleEndianToUint32(&p_buff[idx]);
        idx += 4;        
        float range = Conversion::convertToFloat(&p_buff[idx]);
        idx += 4;
        float velocity = Conversion::convertToFloat(&p_buff[idx]);          
        idx += 4;
        float azimuth = Conversion::convertToFloat(&p_buff[idx]);
        idx += 4;
        float elevation = Conversion::convertToFloat(&p_buff[idx]);
        idx += 4;
        float amplitude = Conversion::convertToFloat(&p_buff[idx]);
        idx += 4;

        // RCLCPP_INFO(rclcpp::get_logger("point_cloud2"), "index %u range %f velocity %f azimuth %f elevation %f amplitude %f", 
        //                                                 index, range, velocity, azimuth, elevation, amplitude); 

        // Convert to Cartesian coordinates
        float x = range * std::cos(elevation * deg2rad) * std::cos(azimuth * deg2rad);
        float y = range * std::cos(elevation * deg2rad) * std::sin(azimuth * deg2rad);
        float z = range * std::sin(elevation * deg2rad);        
        float intensity = amplitude;

        // Copy the data into the PointCloud2 data array
        uint8_t* ptr = &cloud_msg.data[i * cloud_msg.point_step];
        memcpy(ptr + cloud_msg.fields[0].offset, &x, sizeof(float));
        memcpy(ptr + cloud_msg.fields[1].offset, &y, sizeof(float));
        memcpy(ptr + cloud_msg.fields[2].offset, &z, sizeof(float));
        memcpy(ptr + cloud_msg.fields[3].offset, &intensity, sizeof(float));
    }
        
}

void MessageParser::makeRadarScanMssg(uint8_t *p_buff, radar_msgs::msg::RadarScan& radar_scan_msg) {
    uint32_t idx = 0;
    tsPacketHeader header = {};
    std::stringstream ss;

    idx += 4;
    header.ui32UID = Conversion::littleEndianToUint32(&p_buff[idx]);
    idx += 4;
    header.ui32TS = Conversion::littleEndianToUint32(&p_buff[idx]);        
    idx += 4;
    header.ui32TN = Conversion::littleEndianToUint32(&p_buff[idx]);
    idx += 4;
    header.ui32FN = Conversion::littleEndianToUint32(&p_buff[idx]);
    idx += 4;
    header.f32CT = Conversion::convertToFloat(&p_buff[idx]);
    idx += 4;
    header.ui32TPN = Conversion::littleEndianToUint32(&p_buff[idx]);
    idx += 4;
    header.ui32PN = Conversion::littleEndianToUint32(&p_buff[idx]);
    idx += 4;
    header.ui16TPCKN = Conversion::littleEndianToUint16(&p_buff[idx]);
    idx += 2;
    header.ui16PCKN = Conversion::littleEndianToUint16(&p_buff[idx]);
    idx += 2;

    if(header.ui32PN > 60){ 
        RCLCPP_ERROR(rclcpp::get_logger("radar_scan"), "Failed to decode RadarScanMssg ui32PN: %u", header.ui32PN);               
        return;
    }

   // https://github.com/ros2/common_interfaces/blob/rolling/std_msgs/msg/Header.msg
    //sequence_id_ = header.ui32FN; 
    ss << std::hex << header.ui32UID;
    frame_id_ = Util::readFrameIdFromYaml(ss.str());
    if(frame_id_.empty())
        frame_id_ = ss.str();

    stamp_tv_sec_ = header.ui32TS;
    stamp_tv_nsec_ = header.ui32TN;

    RCLCPP_INFO(rclcpp::get_logger("radar_scan"), "UID %08x FN %u TPN %u PN %u TPCKN %u PCKN %u", 
                                                    header.ui32UID, header.ui32FN, header.ui32TPN, header.ui32PN, header.ui16TPCKN, header.ui16PCKN); 

    radar_scan_msg.header.frame_id = frame_id_;
    radar_scan_msg.header.stamp.sec = stamp_tv_sec_;
    radar_scan_msg.header.stamp.nanosec = stamp_tv_nsec_;

    //Point cloud
    for(uint32_t i = 0; i < header.ui32PN; i++)
    {
        radar_msgs::msg::RadarReturn return_msg;
        uint32_t index = Conversion::littleEndianToUint32(&p_buff[idx]);
        idx += 4;           
        return_msg.range = Conversion::convertToFloat(&p_buff[idx]);
        idx += 4;
        return_msg.doppler_velocity = Conversion::convertToFloat(&p_buff[idx]);
        idx += 4;		    
        return_msg.azimuth = Conversion::convertToFloat(&p_buff[idx]);
        idx += 4;
        return_msg.elevation = Conversion::convertToFloat(&p_buff[idx]);
        idx += 4;
        return_msg.amplitude = Conversion::convertToFloat(&p_buff[idx]);
        idx += 4;

        // RCLCPP_INFO(rclcpp::get_logger("RadarScan"), "index %u range %f velocity %f azimuth %f elevation %f amplitude %f", 
        //                                                 index, return_msg.range, return_msg.doppler_velocity, return_msg.azimuth, return_msg.elevation, return_msg.amplitude); 

        radar_scan_msg.returns.push_back(return_msg);
    }    
        
}

void MessageParser::makeRadarTracksMssg(uint8_t *p_buff, radar_msgs::msg::RadarTracks &radar_tracks_msg) {
    uint32_t idx = 0;
    tsPacketHeader header = {};
    std::stringstream ss;

    idx += 4;
    header.ui32UID = Conversion::littleEndianToUint32(&p_buff[idx]);
    ss << std::hex << header.ui32UID;
    frame_id_ = ss.str();

    radar_tracks_msg.header.frame_id = frame_id_;	
    radar_tracks_msg.header.stamp.sec = stamp_tv_sec_;	
    radar_tracks_msg.header.stamp.nanosec = stamp_tv_nsec_;

    RCLCPP_INFO(rclcpp::get_logger("radar_tracks_msg"), "radar tracks msg message frame_id: %s", frame_id_.c_str()); 

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
}

void MessageParser::parseRadarData(uint8_t *p_buff, uint32_t *message_type, 
    #if (POINT_CLOUD2)
    sensor_msgs::msg::PointCloud2& radar_cloud_msg,
    #else
    radar_msgs::msg::RadarScan& radar_scan_msg,
    #endif
    radar_msgs::msg::RadarTracks& radar_tracks_msg) 
{

    uint32_t id = Conversion::littleEndianToUint32(&p_buff[0]);
    *message_type = id;

    if(id == HEADER_SCAN) {
#if (POINT_CLOUD2)
        makeRadarPointCloud2Mssg(p_buff, radar_cloud_msg);  
#else        
        makeRadarScanMssg(p_buff, radar_scan_msg);       
#endif                         
    } else if(id == HEADER_TRACK) {
        makeRadarTracksMssg(p_buff, radar_tracks_msg);           
    } else if(id == HEADER_MON) {
        RCLCPP_INFO(rclcpp::get_logger("MessageParser"), "HEADER_MON message");         
    }  else {
        RCLCPP_INFO(rclcpp::get_logger("MessageParser"), "Failed to decode message id: %08x", id);           
    }
}

}
