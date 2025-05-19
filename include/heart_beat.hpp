#ifndef HEART_BEAT_INCLUDE_H
#define HEART_BEAT_INCLUDE_H

#include <string>
#include <atomic>
#include <thread>
#include <mutex>
#include <cstring>
#include <cstdint>
#include <netinet/in.h>
#include <unordered_map>
#include "rclcpp/rclcpp.hpp"

#include "flatbuffers/flatbuffers.h"

namespace au_4d_radar
{
    class device_au_radar_node;

    class Heartbeat {

    public:
        static Heartbeat& getInstance(device_au_radar_node* node = nullptr);
        explicit Heartbeat(device_au_radar_node* node);
        
        ~Heartbeat();

        void start();
        void stop();

        std::string getClientIP(const std::string& hostname);
        bool connectionStatus(const std::string& hostname);

    private:
        Heartbeat() = delete;
        Heartbeat(const Heartbeat&) = delete;
        Heartbeat& operator=(const Heartbeat&) = delete;
        bool initialize();
        void checkConnectionLoss();
        std::string readFromYaml(const std::string& key);
        bool isValidIPAddress(const std::string& ip);
        std::string inAddrToString(in_addr_t addr);
        void handleClientMessages();
        void setClientIp(const std::string& hostname, const std::string& ip);
        void processRequestConnection(const uint8_t* buffer, const std::string& receivedIp, socklen_t len);
        void processHeartbeatMessage(const uint8_t* buffer, const std::string& receivedIp);

        int recv_sockfd;
        int send_sockfd;
        struct sockaddr_in recv_server_addr, send_server_addr;
        std::string clientHostname;
        std::thread receiverThread;
        std::mutex mtx;
        std::atomic<bool> running;
        std::string configFilePath;
        std::unordered_map<std::string, std::string> clientIpMap;
        std::mutex map_mutex;
        std::unordered_map<std::string, bool> connectionMap;

        std::unordered_map<std::string, uint64_t> last_heartbeat_ts_;
        rclcpp::TimerBase::SharedPtr heartbeat_check_timer_;

        flatbuffers::FlatBufferBuilder builder_;

        device_au_radar_node* radar_node_;
    };

} // namespace au_4d_radar

#endif // HEART_BEAT_INCLUDE_H