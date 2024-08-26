/**
 * @file heart_beat.hpp
 * @author Antonio Ko(antonioko@au-sensor.com)
 * @brief 
 * @version 0.1
 * @date 2024-08-05
 * 
 * @copyright Copyright AU (c) 2024
 * 
 */

#ifndef HEART_BEAT_INCLUDE_H
#define HEART_BEAT_INCLUDE_H

#include <string>
#include <atomic>
#include <thread>
#include <mutex>
#include <cstring>
#include <cstdint>
#include <netinet/in.h>

#include "flatbuffers/flatbuffers.h"

namespace au_4d_radar
{
    class device_au_radar_node;

    class Heartbeat {
    public:
        Heartbeat(device_au_radar_node* node);
        Heartbeat() : Heartbeat(nullptr) {}
        ~Heartbeat();

        void start();
        void stop();
   
        std::string getClientIP();
        bool connectionStatus();

    private:
        bool initialize();
        std::string readFromYaml(const std::string& key);
        bool isValidIPAddress(const std::string& ip);
        std::string inAddrToString(in_addr_t addr);
        void handleClientMessages();
        void setClientIp(const std::string& newIp);     
        void processRequestConnection(const uint8_t* buffer, const std::string& receivedIp, socklen_t len);
        void processHeartbeatMessage(const uint8_t* buffer, const std::string& receivedIp);                

        int recv_sockfd;
        int send_sockfd;
        struct sockaddr_in recv_server_addr, send_server_addr;
        static std::string clientIp;
        std::string clientHostname;
        std::thread receiverThread;
        std::mutex mtx;
        std::atomic<bool> running;
        std::atomic<bool> connected;    
        std::string configFilePath;

       flatbuffers::FlatBufferBuilder builder;                 

        device_au_radar_node* radar_node_;
    };
}

#endif // HEART_BEAT_INCLUDE_H