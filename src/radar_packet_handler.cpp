/**
 * @file radar_packet_handler.cpp
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
#define MSG_TYPE_OFFSET 4 // offset for HeaderType
const size_t MAX_QUEUE_SIZE = 1000;

enum HeaderType {
    HEADER_SCAN = 0x5343414e,
    HEADER_TRACK = 0x54524143,
    HEADER_MON = 0x4d4f4e49
};

namespace au_4d_radar
{

RadarPacketHandler::RadarPacketHandler(device_au_radar_node* node)
    :  rd_sockfd(-1), radar_node_(node), receive_running(true), process_running(true), process_runnings(true)  { }

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
    process_runnings.store(false);

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
    point_cloud2_setting_ = YamlParser::readPointCloud2Setting("POINT_CLOUD2");
    message_number_ = YamlParser::readMessageNumber("MESSAGE_NUMBER");

    rd_sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (rd_sockfd < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("RadarPacketHandler"), "Socket creation failed");
        return false;
    }

    memset(&server_addr_, 0, sizeof(server_addr_));
    server_addr_.sin_family      = AF_INET;
    server_addr_.sin_port        = htons(TARGET_PORT);
    server_addr_.sin_addr.s_addr = htonl(INADDR_ANY);

    int optval = 1;
    if (setsockopt(rd_sockfd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval)) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("RadarPacketHandler"), "Set socket option SO_REUSEADDR failed");
        close(rd_sockfd);
        rd_sockfd = -1;
        return false;
    }

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

    return true;
}

void RadarPacketHandler::receiveMessagesTwoQueues() {
    socklen_t addr_len = sizeof(server_addr_);

    while (receive_running.load()) {
        std::vector<uint8_t> buffer(BUFFER_SIZE);
        int n = recvfrom(rd_sockfd, buffer.data(), buffer.size(), MSG_DONTWAIT, (struct sockaddr*)&server_addr_, &addr_len);
        if (n < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                RCLCPP_ERROR(rclcpp::get_logger("receiveMessagesTwoQueues"), "recvfrom failed");
            }
            usleep(1000);
            continue;
        } else if (n < static_cast<int>(mTsPacketHeaderSize) || n >= BUFFER_SIZE) {
            RCLCPP_INFO(rclcpp::get_logger("receiveMessagesTwoQueues"), "Invalid message size: %d bytes", n);
            continue;
        }

        uint32_t unique_id = Conversion::littleEndianToUint32(&buffer[MSG_TYPE_OFFSET]);
        if (!YamlParser::checkValidFrameId(unique_id)) {
            RCLCPP_INFO(rclcpp::get_logger("receiveMessagesTwoQueues"), "Invalid FrameId: %08x", unique_id);
            continue;
        }

        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (message_queue_.size() >= MAX_QUEUE_SIZE) {
                message_queue_.pop();
                RCLCPP_ERROR(rclcpp::get_logger("receiveMessagesTwoQueues"), "Message queue is full, discarding oldest message");
            }
            message_queue_.emplace(buffer.begin(), buffer.begin() + n);
        }

        queue_cv_.notify_one();
    }
}

void RadarPacketHandler::processMessages() {
    std::vector<uint8_t> buffer(BUFFER_SIZE);
    while (process_running.load()) {
        // std::vector<uint8_t> buffer(BUFFER_SIZE);
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [this] { return !message_queue_.empty() || !process_running.load(); });

            if (!process_running.load()) {
                break;
            }

            if (message_queue_.front().size() < mTsPacketHeaderSize || message_queue_.front().size() >= BUFFER_SIZE) {
                RCLCPP_WARN(rclcpp::get_logger("processMessages"), "Invalid message size detected and discarded.");
                message_queue_.pop();
                continue;
            }

            buffer = std::move(message_queue_.front());
            message_queue_.pop();
        }

        uint32_t unique_id = Conversion::littleEndianToUint32(&buffer[MSG_TYPE_OFFSET]);

        {
            std::lock_guard<std::mutex> lock(client_queue_mutex_);
            if (client_message_queues_[unique_id].size() >= MAX_QUEUE_SIZE) {
                client_message_queues_[unique_id].pop();
                RCLCPP_ERROR(rclcpp::get_logger("processMessages"), "Client message queue is full, discarding oldest message");
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
        buffer.clear();
        client_queue_cvs_[unique_id].notify_one();
    }
}

void RadarPacketHandler::processClientMessages(uint32_t unique_id) {
    radar_msgs::msg::RadarScan radar_scan_msg;
    sensor_msgs::msg::PointCloud2 radar_cloud_msg;
    radar_msgs::msg::RadarTracks radar_tracks_msg;
    std::deque<sensor_msgs::msg::PointCloud2> radar_cloud_buffer;

    std::vector<uint8_t> buffer(BUFFER_SIZE);

    while (process_runnings.load()) {
        // std::vector<uint8_t> buffer(BUFFER_SIZE);
        {
            std::unique_lock<std::mutex> lock(client_queue_mutex_);
            client_queue_cvs_[unique_id].wait(lock, [this, &unique_id] {
                return !client_message_queues_[unique_id].empty() || !process_runnings.load();
            });

            if (!process_runnings.load()) {
                break;
            }

            buffer = std::move(client_message_queues_[unique_id].front());
            client_message_queues_[unique_id].pop();
        }

        uint32_t msg_type = Conversion::littleEndianToUint32(buffer.data());

        switch (msg_type) {
            case HeaderType::HEADER_SCAN:
                handleRadarScanMessage(buffer, radar_scan_msg, radar_cloud_msg, radar_cloud_buffer);
                break;
            case HeaderType::HEADER_TRACK:
                //handleRadarTrackMessage(buffer, radar_tracks_msg);
                break;
            default:
                RCLCPP_WARN(rclcpp::get_logger("processClientMessages"), "Unknown message type: %08x", msg_type);
                break;
        }
        buffer.clear();
    }
}

void RadarPacketHandler::handleRadarScanMessage(std::vector<uint8_t>& buffer, radar_msgs::msg::RadarScan& radar_scan_msg,
        sensor_msgs::msg::PointCloud2& radar_cloud_msg, std::deque<sensor_msgs::msg::PointCloud2>& radar_cloud_buffer) {

    {
        bool completeRadarScanMsg = false;
        {
            std::lock_guard<std::mutex> lock(parse_mutex_);
            message_parser_.parseRadarScanMsg(&buffer[MSG_TYPE_OFFSET], radar_scan_msg, completeRadarScanMsg);
        }

        if (completeRadarScanMsg) {
            std::lock_guard<std::mutex> lock(publish_mutex_);
            radar_node_->publishRadarScanMsg(radar_scan_msg);
            radar_scan_msg.returns.clear();
        }
    }

    if (point_cloud2_setting_) {
        bool completePointCloud2Msg = false;
        {
            std::lock_guard<std::mutex> lock(parse_mutex_);
            message_parser_.parsePointCloud2Msg(&buffer[MSG_TYPE_OFFSET], radar_cloud_msg, completePointCloud2Msg);
        }

        if (completePointCloud2Msg) {
            std::lock_guard<std::mutex> lock(publish_mutex_);
            sensor_msgs::msg::PointCloud2 multiple_cloud_messages;
            assemblePointCloud(radar_cloud_buffer, radar_cloud_msg, multiple_cloud_messages);

            uint32_t time_sync_cloud = radar_cloud_msg.header.stamp.nanosec / 10000000;
            RCLCPP_INFO(rclcpp::get_logger("handleRadarScanMessage"), "id %s 50ms %02u", radar_cloud_msg.header.frame_id.c_str(), time_sync_cloud);

            if (isNewTimeSync(time_sync_cloud)) {
                radar_node_->publishRadarPointCloud2(radar_cloud_msgs);
                radar_cloud_msgs.data.clear();
                radar_cloud_msgs = std::move(multiple_cloud_messages);
                radar_cloud_msgs.header.frame_id = "RADARS";
            } else {
                mergePointCloud(multiple_cloud_messages, radar_cloud_msgs);
            }
            radar_cloud_msg.data.clear();
        }
    }
}

/**
 * @brief function to assemble point cloud messages into a single message
 *
 * @param radar_cloud_buffer
 * @param radar_cloud_msg
 * @param multiple_cloud_messages
 */
void RadarPacketHandler::assemblePointCloud(std::deque<sensor_msgs::msg::PointCloud2>& radar_cloud_buffer,
        const sensor_msgs::msg::PointCloud2& radar_cloud_msg, sensor_msgs::msg::PointCloud2& multiple_cloud_messages) {

    if (message_number_ > 1) {
        radar_cloud_buffer.push_back(radar_cloud_msg);
        if (radar_cloud_buffer.size() > message_number_) {
            radar_cloud_buffer.pop_front();
        }

        multiple_cloud_messages.width = 0;
        multiple_cloud_messages.height = 1;
        multiple_cloud_messages.is_dense = true;
        multiple_cloud_messages.is_bigendian = false;
        multiple_cloud_messages.point_step = radar_cloud_msg.point_step;
        multiple_cloud_messages.fields = radar_cloud_msg.fields;
        multiple_cloud_messages.header = radar_cloud_msg.header;

        for (const auto& msg : radar_cloud_buffer) {
            multiple_cloud_messages.width += msg.width;
            multiple_cloud_messages.data.insert(multiple_cloud_messages.data.end(),
                                                std::make_move_iterator(msg.data.begin()),
                                                std::make_move_iterator(msg.data.end()));
        }
        multiple_cloud_messages.row_step = multiple_cloud_messages.point_step * multiple_cloud_messages.width;
    } else {
        multiple_cloud_messages = radar_cloud_msg;
    }
}

/**
 * @brief function to merge multiple point cloud messages
 *
 * @param multiple_cloud_messages
 * @param radar_cloud_msgs
 */
void RadarPacketHandler::mergePointCloud(const sensor_msgs::msg::PointCloud2& multiple_cloud_messages,
        sensor_msgs::msg::PointCloud2& radar_cloud_msgs) {

    radar_cloud_msgs.width += multiple_cloud_messages.width;
    radar_cloud_msgs.row_step += multiple_cloud_messages.row_step;
    radar_cloud_msgs.data.insert(radar_cloud_msgs.data.end(),
                                 std::make_move_iterator(multiple_cloud_messages.data.begin()),
                                 std::make_move_iterator(multiple_cloud_messages.data.end()));
}

bool RadarPacketHandler::isNewTimeSync(uint32_t time_sync_cloud) {
    bool isNew = (time_sync_pre_cloud_ != time_sync_cloud);
    time_sync_pre_cloud_ = time_sync_cloud;
    return isNew;
}

void RadarPacketHandler::handleRadarTrackMessage(std::vector<uint8_t>& buffer, radar_msgs::msg::RadarTracks& radar_tracks_msg) {
    bool completeRadarTrackMsg = false;
    {
        std::lock_guard<std::mutex> lock(parse_mutex_);
        message_parser_.parseRadarTrackMsg(&buffer[MSG_TYPE_OFFSET], radar_tracks_msg, completeRadarTrackMsg);
    }
    if (completeRadarTrackMsg) {
        std::lock_guard<std::mutex> lock(publish_mutex_);
        radar_node_->publishRadarTrackMsg(radar_tracks_msg);
        radar_tracks_msg.tracks.clear();
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
