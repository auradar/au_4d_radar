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
// #include "radar_packet_handler.hpp"

#define TARGET_PORT 7778
#define BUFFER_SIZE 1500

namespace au_4d_radar
{

RadarPacketHandler::RadarPacketHandler(device_au_radar_node* node)
    : rd_sockfd(-1), radar_running(true), radar_node_(node) {
    memset(&client_addr_, 0, sizeof(client_addr_));
}

RadarPacketHandler::~RadarPacketHandler() {
    stop();
}

void RadarPacketHandler::start() {
    if (initialize()) {
        thread_ = std::thread(&RadarPacketHandler::receive_messages, this);
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
    client_ip = DEFAULT_IP; //radar_node_->heart_beat_.getClientIP();

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

void RadarPacketHandler::receive_messages() {
    radar_msgs::msg::RadarScan radar_scan_msg;
    radar_msgs::msg::RadarTracks radar_tracks_msg;
    uint32_t message_type;
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
        }
        buffer[n] = '\0';

        message_parser_.parse_radar_data(buffer, &message_type, radar_scan_msg, radar_tracks_msg);
        radar_node_->publishRadarData(message_type, radar_scan_msg, radar_tracks_msg);
    }
}

int RadarPacketHandler::send_messages(const char* msg, const char* addr) {
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
