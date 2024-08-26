/**
 * @file au_4d_radar.cpp
 * @author kisoo.kim@au-sensor.com, antonioko@au-sensor.com
 * @brief 
 * @version 0.2
 * @date 2024-03-29
 * 
 * @copyright Copyright AU (c) 2024
 * 
 */

#include <iostream>
#include "au_4d_radar/uuid_helper.hpp"
#include "au_4d_radar/cm.hpp"
#include "au_4d_radar/socket.hpp"


#define PUB_TIME 	40ms
#define MON_TIME 	3000ms

#define UDP_MTU		1500

#define HEADER_SCAN 			0x5343414e 
#define HEADER_TRACK 			0x54524143
#define HEADER_MON 				0x4d4f4e49


using namespace std::chrono_literals;

namespace au_4d_radar
{

typedef struct
{
    uint32_t ui32SB;		/* header */
    uint32_t uniq_id;	
  	uint32_t tv_sec;
    uint32_t tv_nsec;
    uint32_t ui32FN;		/* frame number */
    float f32CT;			/* cycle time */
    uint32_t ui32TPN;		/* total point number */
    uint32_t ui32PN;		/* current point number */
    uint16_t ui16TPCKN;		/* total packet number */
    uint16_t ui16PCKN;		/* current packet number */

}tsPacketHeader;

device_au_radar_node::device_au_radar_node(const rclcpp::NodeOptions & options)
: Node("device_au_radar_node", options), count_(0)
{
  // std::cout << "device_au_radar_node.." << std::endl;
  
  timer_ = this->create_wall_timer(
      PUB_TIME, 
      std::bind(&device_au_radar_node::publish, this));

  timer_mon_ = this->create_wall_timer(
      MON_TIME, 
      std::bind(&device_au_radar_node::monitor, this));

  pub_radar_scan = this->create_publisher<radar_msgs::msg::RadarScan>(
                  "/device/au/radar/scan",
                  rclcpp::SensorDataQoS());

  pub_radar_track = this->create_publisher<radar_msgs::msg::RadarTracks>(
                  "/device/au/radar/track",
                  rclcpp::SensorDataQoS());

  pub_radar_mon = this->create_publisher<mon_msgs::msg::RadarHealth>(
                    "/device/au/radar/status",
                    rclcpp::SensorDataQoS());

  initRadar();

  RCLCPP_INFO(rclcpp::get_logger("device_au_radar_node"), "Start AU 4D Radar Driver Node");

}


void device_au_radar_node::interruptHandler(int sig)
{
    RCLCPP_ERROR(rclcpp::get_logger("interruptHandler"), "signum=%d", sig);

    if(sig==SIGINT || sig==SIGHUP || sig==SIGKILL || sig==SIGSEGV || sig==SIGTERM) {
        // exit

      CommThread comm;

      comm.sendCmdtoRadar("ST");

      comm.stop();

      exit(0);
    }

}
//==============================================================================
template<typename Param>
void device_au_radar_node::get_param(rclcpp::Node::SharedPtr nh, const std::string& name, Param& variable)
{
  using var_type = std::remove_reference_t<decltype(variable)>;

  // check if param is declared earlier
  if (!nh->has_parameter(name))
    // use default variable if user declared var is not avail
    variable = nh->declare_parameter<var_type>(name, variable);
  else
    nh->get_parameter(name, variable);

}

void device_au_radar_node::publish()
{
  uint8_t buffer[UDP_MTU] = {0};
  int32_t len = 0;
  
  if((len = soc_recv((char *)buffer, UDP_MTU)) > 0)
  {
    parse_radar_data(buffer);
    // publish_radar_data();
  }
}

void device_au_radar_node::monitor()
{
  	static uint32_t temp_cnt(0);

	mon_msgs::msg::RadarHealth radar_health_msg;

	radar_health_msg.status  = temp_cnt++;

	std::cout << "publish radar health msgs:"<<radar_health_msg.status << std::endl;

	pub_radar_mon->publish(radar_health_msg);  
    CommThread::sendCmdtoRadar("SS");	
}

// int main(int argc, char ** argv)
int device_au_radar_node::initRadar(void)
{
  CommThread comm;

  signal(SIGINT, interruptHandler);
  signal(SIGHUP, interruptHandler);
  signal(SIGKILL, interruptHandler);
  signal(SIGSEGV, interruptHandler);
  signal(SIGTERM, interruptHandler);

  comm.init();
  comm.sendCmdtoRadar("SS");

    return 0;
}

float device_au_radar_node::convert_to_float(uint8_t * pbuf)
{
	return *((float *)pbuf);
}

uint32_t device_au_radar_node::convert_to_uint32(uint8_t * pbuf)
{
    return (uint32_t)((uint32_t)pbuf[3]<<24|(uint32_t)pbuf[2]<<16|(uint32_t)pbuf[1]<<8|pbuf[0]);
}

uint16_t device_au_radar_node::convetr_to_uint16(uint8_t * pbuf)
{
    return (uint16_t)((uint16_t)pbuf[1]<<8|pbuf[0]);
}

void device_au_radar_node::parse_radar_data(uint8_t * p_buff)
{
	uint32_t idx = 0;
	
	tsPacketHeader header;
	header = {};
    std::stringstream ss;

	// uint32_t id = p_buff[0]<<24|p_buff[1]<<16|p_buff[2]<<8|p_buff[3];
    uint32_t id = convert_to_uint32(&p_buff[0]);
	
	if(id  == HEADER_SCAN)
	{
		
		idx += 4;
        header.uniq_id = convert_to_uint32(&p_buff[idx]);
        idx += 4;		
		header.tv_sec = 	  convert_to_uint32(&p_buff[idx]);
		idx += 4;
		header.tv_nsec =  	convert_to_uint32(&p_buff[idx]);
		idx += 4;
		header.ui32FN = 	  convert_to_uint32(&p_buff[idx]);
		idx += 4;
		header.f32CT =		  convert_to_float(&p_buff[idx]);
		idx += 4;
		header.ui32TPN = 	  convert_to_uint32(&p_buff[idx]);
		idx += 4;
		header.ui32PN = 	  convert_to_uint32(&p_buff[idx]);
		idx += 4;
		header.ui16TPCKN = 	convetr_to_uint16(&p_buff[idx]);
		idx += 2;
		header.ui16PCKN = 	convetr_to_uint16(&p_buff[idx]);
		idx += 2;

        if(header.ui32PN > 60){ // 60
            std::cerr << "Failed to decode parse_radar_data." << " header.ui32PN: " << header.ui32PN << std::endl;
            return;
        }

        // https://github.com/ros2/common_interfaces/blob/rolling/std_msgs/msg/Header.msg
        //sequence_id_ = header.ui32FN; 
        ss << std::hex << header.uniq_id;
        frame_id_ = ss.str();
        stamp_tv_sec_ = header.tv_sec;
        stamp_tv_nsec_ = header.tv_nsec;

		radar_scan_msg.header.frame_id = frame_id_;		//radar coordinate 
		radar_scan_msg.header.stamp.sec = stamp_tv_sec_;
		radar_scan_msg.header.stamp.nanosec = stamp_tv_nsec_;
		
		//Point cloud
		for(uint32_t i = 0; i < header.ui32PN; i++)
		{

			radar_msgs::msg::RadarReturn return_msg;
			
			return_msg.range = convert_to_float(&p_buff[idx]);
			idx += 4;
			return_msg.doppler_velocity = convert_to_float(&p_buff[idx]);
			idx += 4;		    
			return_msg.azimuth = convert_to_float(&p_buff[idx]);
			idx += 4;
			return_msg.elevation = convert_to_float(&p_buff[idx]);
			idx += 4;
			return_msg.amplitude = convert_to_float(&p_buff[idx]);
			idx += 4;

			radar_scan_msg.returns.push_back(return_msg);
		}		

  	pub_radar_scan->publish(radar_scan_msg);    

	}
	else if(id  == HEADER_TRACK)
	{

		radar_tracks_msg.header.frame_id = frame_id_;	
		radar_tracks_msg.header.stamp.sec = stamp_tv_sec_;	
		radar_tracks_msg.header.stamp.nanosec = stamp_tv_nsec_;

		//Tracking
		for(int i = 0; i < 3; i++)
		{
			radar_msgs::msg::RadarTrack radar_data_msg;
		
			radar_data_msg.uuid = tier4_autoware_utils::generateUUID();
			radar_data_msg.position.x = 1.0;
			radar_data_msg.position.y = i;
			radar_data_msg.position.z = i;

			radar_data_msg.velocity.x = 1.0;
			radar_data_msg.velocity.y = 2.0;
			radar_data_msg.velocity.z = 3.0;

			radar_data_msg.acceleration.x = 1.0;
			radar_data_msg.acceleration.y = 2.0;
			radar_data_msg.acceleration.z = 3.0;

			radar_data_msg.size.x = 1.0;
			radar_data_msg.size.y = 2.0;
			radar_data_msg.size.z = 3.0;

			radar_data_msg.classification = 1;			
			radar_data_msg.position_covariance[0] = 1.0; //6 different states ( x, y, z, roll, pitch, yaw)

			radar_tracks_msg.tracks.push_back(radar_data_msg);
		
		}

    pub_radar_track->publish(radar_tracks_msg); 

	}
	else if(id  == HEADER_MON)
	{

	}
	else
	{
		std::cout << "recieved msgs id :  " << std::hex << id << std::endl;
	}

    if(id == HEADER_SCAN || id == HEADER_TRACK){
        std::cout << "parse_radar_data::" 
                << " id : " << std::hex << id
                << " frame_id: " << frame_id_
                << " ui32FN: " << header.ui32FN
                << " ui32TPN: " << header.ui32TPN
                << " ui32PN: " << header.ui32PN                
                << " ui16TPCKN: " << header.ui16TPCKN
                << std::endl;
    }

}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(au_4d_radar::device_au_radar_node)
