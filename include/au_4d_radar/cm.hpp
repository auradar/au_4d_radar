/**
 * @file cm.hpp
 * @author Kisoo Kim (kisoo.kim@au-sensor.com)
 * @brief 
 * @version 0.1
 * @date 2024-03-29
 * 
 * @copyright Copyright AU (c) 2024
 * 
 */
 
#ifndef CM_H
#define CM_H


#include <semaphore>
#include <thread>
#include <condition_variable>

#include <rclcpp/rclcpp.hpp>


class CommThread 
{
public:
    CommThread() {
        std::cout << "CommThread object created" << std::endl;
    }

    ~CommThread() {
        std::cout << "CommThread object destroyed" << std::endl;
    }

    void rxRadar();


    void start() {
        thread_radar = std::thread(&CommThread::rxRadar, this);

       	join();
    }

    void join() {
        if (thread_radar.joinable()) {
            thread_radar.join();
        }

    }

    void init();

    void sendCmdtoRadar(const char * msg);
    

private:
    std::thread thread_radar;
    std::thread thread_pub;

};

#endif

