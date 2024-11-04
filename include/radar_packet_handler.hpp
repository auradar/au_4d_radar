/**
 * @file radar_packet_handler.hpp
 * @author Antonio Ko(antonioko@au-sensor.com)
 * @brief Implementation of the radar_data_handler class for processing incoming radar data.
 * @version 1.1
 * @date 2024-09-05
 *
 * @copyright Copyright AU (c) 2024
 *
 */

#ifndef RADAR_PACKET_HANDLER_INCLUDE_H
#define RADAR_PACKET_HANDLER_INCLUDE_H

#include <condition_variable>
#include <string>
#include <atomic>
#include <cstring>
#include <cstdint>
#include <thread>
#include <queue>
#include <mutex>

#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <map>

#include "message_parse.hpp"


namespace au_4d_radar
{
    class device_au_radar_node;

    class RadarPacketHandler
    {
    public:
        RadarPacketHandler(device_au_radar_node* node);
        ~RadarPacketHandler();

        void start();
        void stop();
        int sendMessages(const char* msg, const char* addr);
        std::string getRadarName(uint32_t radar_id);

    private:
        bool initialize();
        void receiveMessagesTwoQueues();
        void processMessages();
        void processClientMessages(uint32_t unique_id);
        void processPerFrameForAllSensor();
        void handleRadarScanMessage(std::vector<uint8_t>& buffer, radar_msgs::msg::RadarScan& radar_scan_msg,
                sensor_msgs::msg::PointCloud2& radar_cloud_msg, std::deque<sensor_msgs::msg::PointCloud2>& radar_cloud_buffer);
        void assemblePointCloud(std::deque<sensor_msgs::msg::PointCloud2>& radar_cloud_buffer,
                const sensor_msgs::msg::PointCloud2& radar_cloud_msg, sensor_msgs::msg::PointCloud2& multiple_cloud_messages);
        void handleRadarTrackMessage(std::vector<uint8_t>& buffer, radar_msgs::msg::RadarTracks& radar_tracks_msg);

        int rd_sockfd;
        device_au_radar_node* radar_node_;
        std::atomic<bool> receive_running;
        std::atomic<bool> process_running;
        std::atomic<bool> process_runnings;
        bool point_cloud2_setting_;
        uint32_t message_number_;
        
        std::thread receive_thread_;
        std::thread process_thread_;

        std::queue<std::vector<uint8_t>> message_queue_;
        std::mutex queue_mutex_;
        std::condition_variable queue_cv_;

        sockaddr_in server_addr_;
        sockaddr_in client_addr_;
        MessageParser message_parser_;

        std::unordered_map<uint32_t, std::thread> client_threads_;
        std::mutex client_threads_mutex_;
        std::unordered_map<uint32_t, std::queue<std::vector<uint8_t>>> client_message_queues_;
        std::mutex client_queue_mutex_;
        std::unordered_map<uint32_t, std::condition_variable> client_queue_cvs_;

        std::mutex publish_mutex_;
        std::mutex parse_mutex_;

        static constexpr size_t mTsPacketHeaderSize = 36UL;

    };
}

#endif  // RADAR_PACKET_HANDLER_INCLUDE_H