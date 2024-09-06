  /**
 * @file radar_data_handler.hpp
 * @author Antonio Ko(antonioko@au-sensor.com)
 * @brief Implementation of the radar_data_handler class for processing incoming radar data.
 * @version 1.0
 * @date 2024-08-23
 * 
 * @copyright Copyright AU (c) 2024
 * 
 */

#ifndef RADAR_DATA_HANDLER_INCLUDE_H
#define RADAR_DATA_HANDLER_INCLUDE_H

#include <string>
#include <atomic>
#include <cstring>
#include <cstdint>
#include <thread>
#include <queue>

#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>


#include "message_parse.hpp" 
// #include "heart_beat.hpp"  

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

    private:
        bool initialize();
        void receiveMessages();
        void processMessages();

        int rd_sockfd;
        bool radar_running;
        bool point_cloud2_setting;        
        std::thread receive_thread_;
        std::vector<std::thread> worker_threads_;
        std::queue<std::vector<uint8_t>> message_queue_;
        std::mutex queue_mutex_;
        std::condition_variable queue_cv_;
        sockaddr_in server_addr_;
        sockaddr_in client_addr_;
        device_au_radar_node* radar_node_;
        MessageParser message_parser_;

        static const int NUM_WORKER_THREADS = 1; 
    };
}

#endif  // RADAR_DATA_HANDLER_INCLUDE_H