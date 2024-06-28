/**
 * @file socket.hpp
 * @author Kisoo Kim (kisoo.kim@au-sensor.com)
 * @brief 
 * @version 0.1
 * @date 2024-03-29
 * 
 * @copyright Copyright AU (c) 2024
 * 
 */

#ifndef SOCKET_H
#define SOCKET_H


int soc_client(void);
int soc_recv(char* rx_buff, uint32_t size);
int soc_send(const char* msg);


 
#endif

