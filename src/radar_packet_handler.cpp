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
    std::cerr << "RadarPacketHandler::~RadarPacketHandler()" << std::endl;
    stop();
}

void RadarPacketHandler::start() {
    if (initialize()) {
        thread_ = std::thread(&RadarPacketHandler::receive_messages, this);
    } else {
        std::cerr << "Initialization failed, start aborted." << std::endl;
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
    std::cout << "RadarPacketHandler::initialize(set) client_ip: " << client_ip << std::endl;

    rd_sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (rd_sockfd < 0) {
        perror("Socket creation failed");
        return false;
    }

    memset(&server_addr_, 0, sizeof(server_addr_));
	server_addr_.sin_family = AF_INET;
	server_addr_.sin_port = htons(TARGET_PORT);	
	server_addr_.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(rd_sockfd, (struct sockaddr*)&server_addr_, sizeof(server_addr_)) < 0) {
        perror("Bind failed");
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

    std::cout << "RadarPacketHandler::receive_messages(set) client_ip: " << inet_ntoa(client_addr_.sin_addr) << std::endl;

    while (radar_running) { 
        if (rd_sockfd < 0) {
            std::cerr << "Socket file descriptor is invalid" << std::endl;
            return;
        }

        int n = recvfrom(rd_sockfd, (uint8_t *)buffer, BUFFER_SIZE, MSG_DONTWAIT, (struct sockaddr*)&client_addr_, &addr_len);
        if (n < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                perror("RadarPacketHandler recvfrom failed");
            }
            continue;
        }
        buffer[n] = '\0';

        //std::cout << "RadarPacketHandler Received message from client_addr: " << inet_ntoa(client_addr_.sin_addr) << " Data Length: " << n << std::endl;

        message_parser_.parse_radar_data(buffer, &message_type, radar_scan_msg, radar_tracks_msg);
        radar_node_->publishRadarData(message_type, radar_scan_msg, radar_tracks_msg);
    }

    std::cout << "RadarPacketHandler::receive_messages Stop !!!!!!!!!!!!" << std::endl;
}

int RadarPacketHandler::send_messages(const char* msg, const char* addr) {
    struct sockaddr_in client_addr;
    memset(&client_addr, 0, sizeof(client_addr));
    client_addr.sin_family = AF_INET;
    client_addr.sin_addr.s_addr = inet_addr(addr);
    client_addr.sin_port = htons(TARGET_PORT-1);

    int send_len = sendto(rd_sockfd, msg, strlen(msg), 0, (struct sockaddr*)&client_addr, sizeof(client_addr));
    if (send_len == -1) {
        std::cerr << "Error: Failed to send message\n";
        return -1;
    }
    std::cout << "Message sent: " << addr << " msg: " << msg << std::endl;

    return 0;
}

}
