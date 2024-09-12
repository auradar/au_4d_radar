
  /**
 * @file radar_data_handler.cpp
 * @author Antonio Ko(antonioko@au-sensor.com)
 * @brief Implementation of the radar_data_handler class for processing incoming radar data.
 * @version 1.1
 * @date 2024-09-11
 * 
 * @copyright Copyright AU (c) 2024
 * 
 */

#include <string.h>
#include <cerrno> 
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <vector>
#include <unordered_map>
#include <condition_variable>

#include "au_4d_radar.hpp"
#include "util/conversion.hpp"
#include "util/yamlParser.hpp"


#define TARGET_PORT 7778
#define BUFFER_SIZE 1460
#define OFFSET 4 // offset for HeaderType
const size_t MAX_QUEUE_SIZE = 1000;

enum HeaderType {
    HEADER_SCAN = 0x5343414e, 
    HEADER_TRACK = 0x54524143,
    HEADER_MON = 0x4d4f4e49    
};

namespace au_4d_radar
{

RadarPacketHandler::RadarPacketHandler(device_au_radar_node* node)
    :  rd_sockfd(-1), radar_node_(node), receive_running(true), process_running(true) { }

RadarPacketHandler::~RadarPacketHandler() {
    stop();
}

void RadarPacketHandler::start() {
    if (initialize()) {
        receive_thread_ = std::thread(&RadarPacketHandler::receiveMessagesTwoQueues, this);
        process_thread_ = std::thread(&RadarPacketHandler::processMessages, this);
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("RadarPacketHandler"), "Initialization failed, start aborted.");     
    }
}

void RadarPacketHandler::stop() {
    receive_running.store(false);
    process_running.store(false);

    if (rd_sockfd >= 0) {
        close(rd_sockfd);
        rd_sockfd = -1;
    }

    queue_cv_.notify_all();

    {
        std::lock_guard<std::mutex> lock(client_threads_mutex_);
        for (auto& pair : client_queue_cvs_) {
            pair.second.notify_all();
        }
    }

    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }

    if (process_thread_.joinable()) {
        process_thread_.join();
    }

    {
        std::lock_guard<std::mutex> lock(client_threads_mutex_);
        for (auto& pair : client_threads_) {
            if (pair.second.joinable()) {
                pair.second.join();
            }
        }
        client_threads_.clear();
        client_queue_cvs_.clear();
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
    server_addr_.sin_family      = AF_INET;
    server_addr_.sin_port        = htons(TARGET_PORT);
    server_addr_.sin_addr.s_addr = htonl(INADDR_ANY); 

    if (bind(rd_sockfd, (struct sockaddr*)&server_addr_, sizeof(server_addr_)) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("RadarPacketHandler"), "Bind failed");         
        close(rd_sockfd);
        rd_sockfd = -1;
        return false;
    }

    memset(&client_addr_, 0, sizeof(client_addr_));
    client_addr_.sin_family      = AF_INET;
    client_addr_.sin_port        = htons(TARGET_PORT - 1);
    client_addr_.sin_addr.s_addr = htonl(INADDR_ANY);

    message_parser_.init();
    return true;
}

void RadarPacketHandler::receiveMessages() {
    std::vector<uint8_t> buffer(BUFFER_SIZE);
    socklen_t addr_len = sizeof(client_addr_);

    while (receive_running.load()) {
        int n = recvfrom(rd_sockfd, buffer.data(), buffer.size(), MSG_DONTWAIT, 
                         (struct sockaddr*)&client_addr_, &addr_len);
        if (n < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                RCLCPP_ERROR(rclcpp::get_logger("RadarPacketHandler"), "recvfrom failed");
            }
            usleep(1000);  
            continue;
        } else if (n >= BUFFER_SIZE) {
            RCLCPP_INFO(rclcpp::get_logger("RadarPacketHandler"), "Message size: %d bytes", n);
            continue;
        }

        uint32_t unique_id = Conversion::littleEndianToUint32(&buffer[OFFSET]);

        {
            std::lock_guard<std::mutex> lock(client_queue_mutex_);
            if (client_message_queues_[unique_id].size() >= MAX_QUEUE_SIZE) {
                client_message_queues_[unique_id].pop();                       
            }
            client_message_queues_[unique_id].push(buffer);
        }

        {
            std::lock_guard<std::mutex> lock(client_threads_mutex_);
            if (client_threads_.find(unique_id) == client_threads_.end()) {
                client_queue_cvs_[unique_id];
                client_threads_[unique_id] = std::thread(&RadarPacketHandler::processClientMessages, this, unique_id);
                client_threads_[unique_id].detach();  
            }
        }

        client_queue_cvs_[unique_id].notify_one();
    }
}

void RadarPacketHandler::receiveMessagesTwoQueues() {
    uint8_t buffer[BUFFER_SIZE];
    socklen_t addr_len = sizeof(client_addr_);

    while (receive_running.load()) {
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
        } else if (n >= BUFFER_SIZE) {
            RCLCPP_INFO(rclcpp::get_logger("RadarPacketHandler"), "Message size: %d bytes", n);
            continue;
        }

        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (message_queue_.size() >= MAX_QUEUE_SIZE) {
                message_queue_.pop();
            }
            message_queue_.emplace(buffer, buffer + n);
        }

        queue_cv_.notify_one();
    }
}

void RadarPacketHandler::processMessages() {
    while (process_running.load()) {
        std::vector<uint8_t> buffer;

        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [this] { return !message_queue_.empty() || !process_running.load(); });

            if (!process_running.load()) {
                std::lock_guard<std::mutex> lock(client_queue_mutex_);
                while (!message_queue_.empty()) {
                    message_queue_.pop();
                }  
                break;
            }

            buffer = std::move(message_queue_.front());
            message_queue_.pop();
        }

        uint32_t unique_id = Conversion::littleEndianToUint32(&buffer[OFFSET]);

        {
            std::lock_guard<std::mutex> lock(client_queue_mutex_);
            if (client_message_queues_[unique_id].size() >= MAX_QUEUE_SIZE) {
                client_message_queues_[unique_id].pop();                       
            }
            client_message_queues_[unique_id].push(buffer);
        }

        {
            std::lock_guard<std::mutex> lock(client_threads_mutex_);
            if (client_threads_.find(unique_id) == client_threads_.end()) {
                client_queue_cvs_[unique_id];  
                client_threads_[unique_id] = std::thread(&RadarPacketHandler::processClientMessages, this, unique_id);
            }
        }

        client_queue_cvs_[unique_id].notify_one(); 
    }
}

void RadarPacketHandler::processClientMessages(uint32_t unique_id) {
    radar_msgs::msg::RadarScan radar_scan_msg;        
    sensor_msgs::msg::PointCloud2 radar_cloud_msg;
    radar_msgs::msg::RadarTracks radar_tracks_msg;

    while (process_running.load()) {
        std::vector<uint8_t> buffer(BUFFER_SIZE);

        {
            std::unique_lock<std::mutex> lock(client_queue_mutex_);
            client_queue_cvs_[unique_id].wait(lock, [this, &unique_id] { 
                return !client_message_queues_[unique_id].empty() || !process_running.load(); 
            });

            if (!process_running.load()) {
                while (!client_message_queues_[unique_id].empty()) {
                    client_message_queues_[unique_id].pop();
                }          
                break;
            }

            buffer = std::move(client_message_queues_[unique_id].front());
            client_message_queues_[unique_id].pop();
        }

        uint32_t msg_type = Conversion::littleEndianToUint32(buffer.data());

        switch (msg_type) {
            case HeaderType::HEADER_SCAN: 
            {
                bool completeRadarScanMsg   = false;
                bool completePointCloud2Msg = false;
                message_parser_.parseRadarScanMsg(&buffer[OFFSET], radar_scan_msg, completeRadarScanMsg);
                if (completeRadarScanMsg) {
                    radar_node_->publishRadarScanMsg(radar_scan_msg);
                    radar_scan_msg.returns.clear();
                }

                if (point_cloud2_setting.load()) {
                    message_parser_.parsePointCloud2Msg(&buffer[OFFSET], radar_cloud_msg, completePointCloud2Msg);
                    if (completePointCloud2Msg) {
                        radar_node_->publishRadarPointCloud2(radar_cloud_msg);
                        radar_cloud_msg.data.clear();
                    }
                }
            }
                break;

            case HeaderType::HEADER_TRACK: 
            {
                bool completeRadarTrackMsg = false;
                message_parser_.parseRadarTrackMsg(&buffer[OFFSET], radar_tracks_msg, completeRadarTrackMsg);
                if (completeRadarTrackMsg) {
                    radar_node_->publishRadarTrackMsg(radar_tracks_msg);
                    radar_tracks_msg.tracks.clear();
                }
            }
                break;

            default: 
                break;
        }
    }
}

int RadarPacketHandler::sendMessages(const char* msg, const char* addr) {
    struct sockaddr_in client_addr;
    memset(&client_addr, 0, sizeof(client_addr));
    client_addr.sin_family      = AF_INET;
    client_addr.sin_addr.s_addr = inet_addr(addr);
    client_addr.sin_port        = htons(TARGET_PORT - 1);

    int send_len = sendto(rd_sockfd, msg, strlen(msg), 0, (struct sockaddr*)&client_addr, sizeof(client_addr));
    if (send_len == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("RadarPacketHandler"), "Failed to send message");
        return -1;
    }
    return 0;
}

}  // namespace au_4d_radar
