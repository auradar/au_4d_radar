
/**
 * @file util.cpp
 * @author Antonio Ko(antonioko@au-sensor.com)
 * @brief 
 * @version 0.1
 * @date 2024-08-05
 * 
 * @copyright Copyright AU (c) 2024
 * 
 */

#include "rclcpp/rclcpp.hpp"
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "util/util.hpp"

std::string Util::readHostnameFromYaml(const std::string& key) {
    try {
        std::string yaml_file_path = ament_index_cpp::get_package_share_directory("au_4d_radar") + "/config/system_info.yaml";        
        YAML::Node config = YAML::LoadFile(yaml_file_path); 

        if (config[key]) {
            return config[key].as<std::string>();
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("readHostnameFromYaml"), "not found in system_info.yaml key: %s", key.c_str());             
            return "";
        }
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("readHostnameFromYaml"), "Error reading YAML file: %s", e.what());          
        return "";
    }
}

std::string Util::readFrameIdFromYaml(const std::string& key) {
    try {
        std::string yaml_file_path = ament_index_cpp::get_package_share_directory("au_4d_radar") + "/config/system_info.yaml";        
        YAML::Node config = YAML::LoadFile(yaml_file_path); 

        if (config["radars"]) {
            YAML::Node radars = config["radars"];
            
            if (radars[key]) {
                return radars[key].as<std::string>();
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("readFrameIdFromYaml"), "frame_id not found in 'radars' section: %s", key.c_str());
                return "";
            }
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("readFrameIdFromYaml"), "'radars' section not found in system_info.yaml");
            return "";
        }
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("readFrameIdFromYaml"), "Error reading YAML file: %s", e.what());          
        return "";
    }
}

