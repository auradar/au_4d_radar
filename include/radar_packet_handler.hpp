  /**
 * @file radar_data_handler.hpp
 * @author Antonio Ko(antonioko@au-sensor.com)
 * @brief Implementation of the radar_data_handler class for processing incoming radar data.
 * @version 0.1
 * @date 2024-08-05
 * 
 * @copyright Copyright AU (c) 2024
 * 
 */

#ifndef RADAR_DATA_HANDLER_INCLUDE_H
#define RADAR_DATA_HANDLER_INCLUDE_H

#include <thread>
#include <netinet/in.h>
#include <string>
#include <atomic>
#include <cstring>
#include <cstdint>

#include <arpa/inet.h>
#include <sys/socket.h>

#include "message_parse.hpp" 
// #include "heart_beat.hpp"  

namespace au_4d_radar
{
    class device_au_radar_node;

    class RadarPacketHandler {
    public:
        RadarPacketHandler(device_au_radar_node* node);
        ~RadarPacketHandler();

        void start();
        void stop();
        int send_messages(const char* msg, const char* addr);

    private:
        bool initialize();
        void receive_messages(); 

        int rd_sockfd;
        struct sockaddr_in server_addr_;
        struct sockaddr_in client_addr_;
        std::string client_ip;
        std::atomic<bool> radar_running;
        std::thread  thread_;
        MessageParser message_parser_;
        device_au_radar_node* radar_node_;
    };
}

#endif  // RADAR_DATA_HANDLER_INCLUDE_H