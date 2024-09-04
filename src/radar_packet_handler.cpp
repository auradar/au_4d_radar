/**
 * @file radar_packet_handler.cpp
 * @author Antonio Ko(antonioko@au-sensor.com)
 * @brief Implementation of the radar_data_handler class for processing incoming radar data.
 * @version 2.1
 * @date 2024-09-04
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
#include <vector>

#include <mutex>
#include <condition_variable>

#include "au_4d_radar.hpp"
#include "util/conversion.hpp"
#include "util/yamlParser.hpp"

#define TARGET_PORT 7778
#define BUFFER_SIZE 1500

enum HeaderType {
    HEADER_SCAN = 0x5343414e, 
    HEADER_TRACK = 0x54524143,
    HEADER_MON = 0x4d4f4e49    
};


namespace au_4d_radar
{

RadarPacketHandler::RadarPacketHandler(device_au_radar_node* node)
    : rd_sockfd(-1), radar_running(true), radar_node_(node) {}

RadarPacketHandler::~RadarPacketHandler() {
    stop();
}

void RadarPacketHandler::start() {
    if (initialize()) {
        receive_thread_ = std::thread(&RadarPacketHandler::receiveMessages, this);

        for (int i = 0; i < NUM_WORKER_THREADS; ++i) {
            worker_threads_.emplace_back(&RadarPacketHandler::processMessages, this);
        }
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

    queue_cv_.notify_all();

    if (receive_thread_.joinable()) {
        receive_thread_.join();
    } 

    for (auto& thread : worker_threads_) {
        if (thread.joinable()) {
            thread.join();
        }
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
    client_addr_.sin_addr.s_addr = htonl(INADDR_ANY);

    return true;
}

void RadarPacketHandler::receiveMessages() {
    uint8_t buffer[BUFFER_SIZE];
    socklen_t addr_len = sizeof(client_addr_);

    while (radar_running) { 
        if (rd_sockfd < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("RadarPacketHandler"), "Socket file descriptor is invalid");              
            return;
        }

        int n = recvfrom(rd_sockfd, buffer, BUFFER_SIZE, MSG_DONTWAIT, (struct sockaddr*)&client_addr_, &addr_len);
        if (n < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                RCLCPP_ERROR(rclcpp::get_logger("RadarPacketHandler"), "recvfrom failed");                 
            }
            usleep(1000); 
            continue;
        } else if (n > BUFFER_SIZE) {
            RCLCPP_ERROR(rclcpp::get_logger("RadarPacketHandler"), "message size exceeds buffer size");                 
            continue;
        }

        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            message_queue_.emplace(buffer, buffer + n);
        }

        queue_cv_.notify_one();
    }
}

void RadarPacketHandler::processMessages() {
    while (radar_running) {
        std::vector<uint8_t> buffer;
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [this] { return !message_queue_.empty() || !radar_running; });

            if (!radar_running && message_queue_.empty()) {
                break;
            }

            buffer = std::move(message_queue_.front());
            message_queue_.pop();
        }

        uint32_t msg_type = Conversion::littleEndianToUint32(buffer.data());
        uint16_t offset = sizeof(msg_type);

        if(msg_type == HeaderType::HEADER_SCAN) { 
            radar_msgs::msg::RadarScan radar_scan_msg;

            message_parser_.parseRadarScanMsg(&buffer[offset], radar_scan_msg);
            radar_node_->publishRadarScanMsg(radar_scan_msg);    

            if(point_cloud2_setting) {
                sensor_msgs::msg::PointCloud2 radar_cloud_msg;                                                     
                message_parser_.parsePointCloud2Msg(&buffer[offset], radar_cloud_msg);
                radar_node_->publishRadarPointCloud2(radar_cloud_msg);    
            }         
        } else if(msg_type == HeaderType::HEADER_TRACK) {
            radar_msgs::msg::RadarTracks radar_tracks_msg;
            message_parser_.parseRadarTrackMsg(&buffer[offset], radar_tracks_msg);
            radar_node_->publishRadarTrackMsg(radar_tracks_msg);           
        } else if(msg_type == HeaderType::HEADER_MON) {
            RCLCPP_INFO(rclcpp::get_logger("RadarPacketHandler"), "HEADER_MON message");         
        } else {
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

} // namespace au_4d_radar
