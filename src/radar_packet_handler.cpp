/**
 * @file radar_packet_handler.cpp
 * @author Antonio Ko(antonioko@au-sensor.com)
 * @brief Implementation of the radar_data_handler class for processing incoming radar data.
 * @version 1.0
 * @date 2024-08-23
 * 
 * @copyright Copyright AU (c) 2024
 * 
 */

#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <thread>

#include "au_4d_radar.hpp"
#include "util/conversion.hpp"
#include "util/yamlParser.hpp"

// #include "radar_packet_handler.hpp"

#define TARGET_PORT 7778
#define BUFFER_SIZE 1500

#define HEADER_SCAN 			0x5343414e 
#define HEADER_TRACK 			0x54524143
#define HEADER_MON 				0x4d4f4e49

namespace au_4d_radar
{

RadarPacketHandler::RadarPacketHandler(device_au_radar_node* node)
    : rd_sockfd(-1), radar_running(true), radar_node_(node) {}

RadarPacketHandler::~RadarPacketHandler() {
    stop();
}

void RadarPacketHandler::start() {
    if (initialize()) {
        thread_ = std::thread(&RadarPacketHandler::receiveMessages, this);
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("RadarPacketHandler"), "Initialization failed, start aborted.");     
    }
}

void RadarPacketHandler::stop() {
    radar_running = false;
    
    if (rd_sockfd >= 0) {
        close(rd_sockfd);
        rd_sockfd = -1;
    }

    if (thread_.joinable()) {
        thread_.join();
    } 
}

bool RadarPacketHandler::initialize() {
    point_cloud2_setting = YamlParser::readPointCloud2Setting("POINT_CLOUD2");

    rd_sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (rd_sockfd < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("RadarPacketHandler"), "Socket creation failed");         
        return false;
    }

    memset(&server_addr_, 0, sizeof(server_addr_));
	server_addr_.sin_family = AF_INET;
	server_addr_.sin_port = htons(TARGET_PORT);	
	server_addr_.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(rd_sockfd, (struct sockaddr*)&server_addr_, sizeof(server_addr_)) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("RadarPacketHandler"), "Bind failed");         
        close(rd_sockfd);
        rd_sockfd = -1;
        return false;
    }

    memset(&client_addr_, 0, sizeof(client_addr_));
	client_addr_.sin_family = AF_INET;
	client_addr_.sin_port = htons(TARGET_PORT-1);
	client_addr_.sin_addr.s_addr = htonl(INADDR_ANY); //inet_addr(DEFAULT_IP);

    return true;
}

void RadarPacketHandler::receiveMessages() {

    sensor_msgs::msg::PointCloud2 radar_cloud_msg;    
    radar_msgs::msg::RadarScan radar_scan_msg;    
    radar_msgs::msg::RadarTracks radar_tracks_msg;
    uint8_t buffer[BUFFER_SIZE];
    socklen_t addr_len = sizeof(client_addr_);

    while (radar_running) { 
        if (rd_sockfd < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("RadarPacketHandler"), "Socket file descriptor is invalid");              
            return;
        }

        int n = recvfrom(rd_sockfd, (uint8_t *)buffer, BUFFER_SIZE, MSG_DONTWAIT, (struct sockaddr*)&client_addr_, &addr_len);
        if (n < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                RCLCPP_ERROR(rclcpp::get_logger("RadarPacketHandler"), "recvfrom failed");                 
            }
            continue;
        } else if (n > BUFFER_SIZE) {
            RCLCPP_ERROR(rclcpp::get_logger("RadarPacketHandler"), "message size exceeds buffer size");                 
            continue;
        } else {
            buffer[n] = '\0';
        }
        
        uint32_t msg_type = Conversion::littleEndianToUint32(buffer);
        uint16_t offset = sizeof(msg_type);
        if(msg_type == HEADER_SCAN) { 
            message_parser_.parseRadarScanMsg(&buffer[offset], radar_scan_msg);
            radar_node_->publishRadarScanMsg(radar_scan_msg);    
            if(point_cloud2_setting) {                                  
                message_parser_.parsePointCloud2Msg(&buffer[offset], radar_cloud_msg);
                radar_node_->publishRadarPointCloud2(radar_cloud_msg);    
            }         
        } else if(msg_type == HEADER_TRACK) {
            message_parser_.parseRadarTrackMsg(&buffer[offset], radar_tracks_msg);
            radar_node_->publishRadarTrackMsg(radar_tracks_msg);           
        } else if(msg_type == HEADER_MON) {
            RCLCPP_INFO(rclcpp::get_logger("RadarPacketHandler"), "HEADER_MON message");         
        }  else {
            RCLCPP_INFO(rclcpp::get_logger("RadarPacketHandler"), "Failed to decode msg_type: %08x", msg_type);           
        }
     
    }
}

int RadarPacketHandler::sendMessages(const char* msg, const char* addr) {
    struct sockaddr_in client_addr;
    memset(&client_addr, 0, sizeof(client_addr));
    client_addr.sin_family = AF_INET;
    client_addr.sin_addr.s_addr = inet_addr(addr);
    client_addr.sin_port = htons(TARGET_PORT-1);

    int send_len = sendto(rd_sockfd, msg, strlen(msg), 0, (struct sockaddr*)&client_addr, sizeof(client_addr));
    if (send_len == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("RadarPacketHandler"), "Failed to send message");         
        return -1;
    }
    //RCLCPP_INFO(rclcpp::get_logger("RadarPacketHandler"), "Message sent: %s msg: %s", addr, msg);      
    return 0;
}

}
