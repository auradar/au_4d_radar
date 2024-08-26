/**
 * @file cm.cpp
 * @author Kisoo Kim (kisoo.kim@au-sensor.com)
 * @brief 
 * @version 0.1
 * @date 2024-03-29
 * 
 * @copyright Copyright AU (c) 2024
 * 
 */
 

#include <iostream>
#include <thread>
#include <chrono>
#include <condition_variable>


#include "au_4d_radar/socket.hpp"
#include "au_4d_radar/cm.hpp"
#include "au_4d_radar/au_4d_radar.hpp"


void CommThread::init()
{
	soc_client();
}

void CommThread::sendCmdtoRadar(const char * msg)
{
	soc_send(msg);
}


void CommThread::rxRadar(void)
{
	

	while(1)
	{


	}

}

