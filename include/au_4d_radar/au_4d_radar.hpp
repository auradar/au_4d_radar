/**
 * @file au_4d_radar.hpp
 * @author kisoo.kim@au-sensor.com
 * @version 0.1
 * @date 2024-04-05
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef AU_4D_RADAR_INCLUDE_AU_4D_RADAR_H
#define AU_4D_RADAR_INCLUDE_AU_4D_RADAR_H

#include <iostream>
#include <cstdlib>
#include <algorithm>
#include <random>
#include <string>
#include <signal.h>
#include <sys/types.h>
#include <boost/uuid/uuid.hpp>


#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <radar_msgs/msg/radar_return.hpp>
#include <radar_msgs/msg/radar_scan.hpp>
#include <radar_msgs/msg/radar_track.hpp>
#include <radar_msgs/msg/radar_tracks.hpp>
#include "mon_msgs/msg/radar_health.hpp"

// using namespace std::chrono_literals;

namespace au_4d_radar
{

class device_au_radar_node : public rclcpp::Node
{
public:
  explicit device_au_radar_node(const rclcpp::NodeOptions & options);

private:
  template<typename Param>
  void get_param(rclcpp::Node::SharedPtr nh, const std::string& name, Param& variable);

  int initRadar(void);
  static void interruptHandler(int sig);
	void publish();    
	void monitor();
  void parse_radar_data(uint8_t * p_buff);

  static float convert_to_float(uint8_t * pbuf);
  static uint32_t convert_to_uint32(uint8_t * pbuf);
  static uint16_t convetr_to_uint16(uint8_t * pbuf);


  std::shared_ptr<rclcpp::Publisher<radar_msgs::msg::RadarScan>> pub_radar_scan;
  std::shared_ptr<rclcpp::Publisher<radar_msgs::msg::RadarTracks>> pub_radar_track;
  std::shared_ptr<rclcpp::Publisher<mon_msgs::msg::RadarHealth>> pub_radar_mon;

	rclcpp::TimerBase::SharedPtr timer_;	
	rclcpp::TimerBase::SharedPtr timer_mon_;    

	size_t count_;

  static uint32_t temp_cnt;
  std::string frame_id;
  uint32_t stamp_tv_sec;
  uint32_t stamp_tv_nsec;

  radar_msgs::msg::RadarTracks radar_tracks_msg;
  radar_msgs::msg::RadarScan radar_scan_msg;


};

}

#endif
