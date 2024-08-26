/**
 * @file heart_beat.cpp
 * @author Antonio Ko(antonioko@au-sensor.com)
 * @brief 
 * @version 0.1
 * @date 2024-08-22
 * 
 * @copyright Copyright AU (c) 2024
 * 
 */

#include <iostream>
#include <arpa/inet.h>
#include <unistd.h>
#include <fstream>
#include <ifaddrs.h>
// #include <netinet/in.h>
#include <sys/socket.h>
#include <netdb.h>

#include <sstream>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "flat/Heartbeat_generated.h"
#include "flat/RequestConnection_generated.h"
#include "flat/ResponseConnection_generated.h"
#include "util/HexDump.hpp"
#include "util/conversion.hpp"

#include "au_4d_radar.hpp"
// #include "heart_beat.hpp"

#define RECEIVE_PORT 59152 // Users can use UDP ports in the range 49152-65535.
#define SEND_PORT 59153
#define BUFFER_SIZE 1472 
#define MSSG_OFFSET 4 // offset for MessageType size(4bytes)

enum MessageType {
    REQUEST_CONNECTION  = 0x41551001,
    RESPONSE_CONNECTION = 0x41551002,
    Heartbeat_MESSAGE   = 0x41551003
};


namespace au_4d_radar
{
std::string Heartbeat::clientIp = DEFAULT_IP;

Heartbeat::Heartbeat(device_au_radar_node* node) 
    : send_sockfd(-1), running(true), connected(false), radar_node_(node) {}

Heartbeat::~Heartbeat() { 
    std::cerr << "Heartbeat::~Heartbeat()" << std::endl;         
    stop(); 
}

bool Heartbeat::initialize() {
    clientHostname = readFromYaml("client_hostname");

    std::cout << "Heartbeat::initialize() clientHostname: " << clientHostname << std::endl;

    recv_sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (recv_sockfd < 0) {
        perror("Socket creation failed");
        return false;
    }

    int optval = 1;
    if (setsockopt(recv_sockfd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval)) < 0) {
        perror("Set socket option SO_REUSEADDR failed");       
        close(recv_sockfd);
        return false;
    }

    send_sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (send_sockfd < 0) {
        perror("Socket creation failed");
        close(recv_sockfd);
        return false;
    }

    memset(&recv_server_addr, 0, sizeof(recv_server_addr));
    recv_server_addr.sin_family      = AF_INET;
    recv_server_addr.sin_port        = htons(RECEIVE_PORT);
    recv_server_addr.sin_addr.s_addr = INADDR_ANY;

    memset(&send_server_addr, 0, sizeof(send_server_addr));
    send_server_addr.sin_family      = AF_INET;
    send_server_addr.sin_port        = htons(SEND_PORT);
    send_server_addr.sin_addr.s_addr = INADDR_BROADCAST;  // Set to broadcast address to send responses

    if (bind(recv_sockfd, (const struct sockaddr *)&recv_server_addr, sizeof(recv_server_addr)) < 0) {
        perror("Bind failed");
        close(recv_sockfd);
        close(send_sockfd);
        return false;
    }

    return true;
}

void Heartbeat::start() {
    if (initialize()) {
        receiverThread = std::thread(&Heartbeat::handleClientMessages, this);
    }
}

void Heartbeat::stop() {
    running = false;
    if (recv_sockfd >= 0) {
        close(recv_sockfd);
        recv_sockfd = -1;
    }

    if (send_sockfd >= 0) {
        close(send_sockfd);
        send_sockfd = -1;
    }

    if (receiverThread.joinable()) {
        receiverThread.join();
    }

    if (receiverThread.joinable()) {
        receiverThread.join();
    }  
}  

bool Heartbeat::connectionStatus(){
    return connected;
}

void Heartbeat::processRequestConnection(const uint8_t* buffer, const std::string& receivedIp, socklen_t len) {
    auto request = AU::GetRequestConnection(&buffer[MSSG_OFFSET]);
    if (!request) {
        std::cerr << "Failed to decode RequestConnection message." << std::endl;
        return;
    }

    std::string receivedHostname = request->client_hostname()->str();
    std::cout << "Request Connection from client_hostname: " << receivedHostname << std::endl;

    if (receivedHostname.starts_with(clientHostname.substr(0, 7))) { 
        builder.Clear();
        auto client_hostname = builder.CreateString(clientHostname);
        auto response        = AU::CreateResponseConnection(builder, builder.CreateString("RESPONSE_CONNECTION"), client_hostname);
        builder.Finish(response);
        size_t buff_size = builder.GetSize() + MSSG_OFFSET;
        std::vector<uint8_t> buff(buff_size);
        Conversion::uint32_to_big_endian(MessageType::RESPONSE_CONNECTION, buff.data()); 
        memcpy(&buff[MSSG_OFFSET], builder.GetBufferPointer(), builder.GetSize());
      
        if (inet_pton(AF_INET, receivedIp.c_str(), &send_server_addr.sin_addr) <= 0) {
            std::cerr << "Invalid IP address format: " << receivedIp << std::endl;
            send_server_addr.sin_addr.s_addr = INADDR_BROADCAST;            
        }        

        sendto(send_sockfd, buff.data(), buff_size, 0, (const struct sockaddr *)&send_server_addr, len);
        std::cout << "Response for request connection sent to: " << receivedIp << std::endl;
    } else {
        std::cout << "Hostname does not match" << std::endl;
    }
}

void Heartbeat::processHeartbeatMessage(const uint8_t* buffer, const std::string& receivedIp) {
    auto Heartbeat = AU::GetHeartbeat(&buffer[MSSG_OFFSET]);
    if (!Heartbeat) {
        std::cerr << "Failed to decode Heartbeat message." << std::endl;
        return;
    }

    std::string HeartbeatHostname = Heartbeat->client_hostname()->str();
    if (clientHostname.starts_with(HeartbeatHostname.substr(0, 7))) {
            mon_msgs::msg::RadarHealth radar_health_msg;            
            time_t raw_time = static_cast<time_t>(Heartbeat->timestamp());
            struct tm* timeinfo = localtime(&raw_time);
            char time_str[64];
            strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", timeinfo);
            setClientIp(receivedIp);
            radar_node_->radar_handler_.send_messages("SS", receivedIp.c_str()); 

            std::cout << "client_hostname: " << HeartbeatHostname
                    << " receivedIp: " << receivedIp
                    << " Status: " << Heartbeat->status()
                    << " Time: " << time_str << std::endl;

            radar_health_msg.client_hostname = HeartbeatHostname;
            radar_health_msg.status = Heartbeat->status();
            radar_health_msg.tv_sec = Heartbeat->timestamp();

            radar_node_->publishHeartbeat(radar_health_msg);  
    }
}

void Heartbeat::handleClientMessages() {
    uint8_t buffer[BUFFER_SIZE];

    while (running) {
        socklen_t len = sizeof(recv_server_addr);
        int       n   = recvfrom(recv_sockfd, buffer, sizeof(buffer), 0, (struct sockaddr *)&recv_server_addr, &len);
        if (n < 0) {
            if (running) {
                perror("recvfrom");
            }
            continue;
        } else if (n > BUFFER_SIZE) {
            std::cerr << "Received message size exceeds buffer size" << std::endl;
            continue;
        }

        std::string receivedIp = inAddrToString(recv_server_addr.sin_addr.s_addr);
        uint32_t mssg_type = Conversion::big_endian_to_uint32(buffer);

        switch (mssg_type) {
            case MessageType::REQUEST_CONNECTION: 
                processRequestConnection(buffer, receivedIp, len);
                break;
            case MessageType::Heartbeat_MESSAGE: 
                processHeartbeatMessage(buffer, receivedIp);
                break;
            default: 
                std::cout << "Unknown message type: " << mssg_type << " receivedIp: " << receivedIp << std::endl;
                std::cout << HexDump(buffer, n) << std::endl;
                break;
        }
    }
}

std::string Heartbeat::readFromYaml(const std::string& key) {
    try {
        std::string yaml_file_path = ament_index_cpp::get_package_share_directory("au_4d_radar") + "/config/system_info.yaml";        
        YAML::Node config = YAML::LoadFile(yaml_file_path); 

        if (config[key]) {
            return config[key].as<std::string>();
        } else {
            std::cerr << key << " not found in system_info.yaml" << std::endl;
            return "";
        }
    } catch (const YAML::Exception& e) {
        std::cerr << "Error reading YAML file: " << e.what() << std::endl;
        return "";
    }
}

bool Heartbeat::isValidIPAddress(const std::string& ip) {
    struct sockaddr_in sa;
    return inet_pton(AF_INET, ip.c_str(), &(sa.sin_addr)) != 0;
}

std::string Heartbeat::inAddrToString(in_addr_t addr) {
    struct in_addr ipAddr;
    ipAddr.s_addr = addr;
    return std::string(inet_ntoa(ipAddr));
}

void Heartbeat::setClientIp(const std::string& newIp) {
    if (clientIp != newIp) {
        clientIp = newIp;   
        connected = true;                   
        std::cout << "Client IP set to: " << clientIp << std::endl;
    }
}

std::string Heartbeat::getClientIP() {
    return clientIp;
}

}