/**
 * @file socket.cpp
 * @author Kisoo Kim (kisoo.kim@au-sensor.com)
 * @brief 
 * @version 0.1
 * @date 2024-03-29
 * 
 * @copyright Copyright AU (c) 2024
 * 
 */

#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"

#define TARGET_PORT	7778
#define TARGET_IP	"192.168.97.6"	// radar Board IP

int udp_socket;

struct sockaddr_in      gLocalSock;
struct sockaddr_in      gRemoteSock;



int soc_client(void)
{

	gLocalSock.sin_family = AF_INET;
	gLocalSock.sin_port = htons(TARGET_PORT);	
	gLocalSock.sin_addr.s_addr = htonl(INADDR_ANY);
	
	gRemoteSock.sin_family = AF_INET;
	gRemoteSock.sin_port = htons(TARGET_PORT-1);
	gRemoteSock.sin_addr.s_addr = inet_addr(TARGET_IP);


	udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (udp_socket == -1) 
	{
        RCLCPP_ERROR(rclcpp::get_logger("RadarPacketHandler"), "Socket creation failed");   
		return -1;
	}


	if (bind(udp_socket, (struct sockaddr *)&gLocalSock, sizeof(gLocalSock)) == 0)
	{
		return -1;
	}

    RCLCPP_INFO(rclcpp::get_logger("RadarPacketHandler"), "connect OK");    

	return 0;
}

int soc_recv(char* rx_buff, uint32_t size)
{
	struct sockaddr_in server_addr;
	uint32_t server_struct_length = sizeof(server_addr);
	int32_t len = 0;

	if((len = recvfrom(udp_socket, (char *)rx_buff, size, MSG_DONTWAIT, (struct sockaddr*)&gRemoteSock, &server_struct_length)) < 0)
	{
		return -1;
	}

	return len;
}

int soc_send(const char* msg)
{
	int send_len = sendto(udp_socket, msg, strlen(msg), 0, (struct sockaddr*)&gRemoteSock, sizeof(struct sockaddr));
	if (send_len == -1) 
	{
        RCLCPP_ERROR(rclcpp::get_logger("socket"), "Error: Failed to send message"); 		
		return -1;
	}
    RCLCPP_INFO(rclcpp::get_logger("RadarPacketHandler"), "Message sent msg: %s", msg);   
	return 0;
}



