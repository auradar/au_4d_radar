
/**
 * @file yamlParser.cpp
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
#include "util/yamlParser.hpp"

std::string YamlParser::readHostname(const std::string& key) {
    try {
        std::string yaml_file_path = ament_index_cpp::get_package_share_directory("au_4d_radar") + "/config/system_info.yaml";        
        YAML::Node config = YAML::LoadFile(yaml_file_path); 

        if (config[key]) {
            return config[key].as<std::string>();
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("readHostname"), "not found in system_info.yaml key: %s", key.c_str());             
            return "";
        }
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("readHostname"), "Error reading YAML file: %s", e.what());          
        return "";
    }
}


bool YamlParser::readPointCloud2Setting(const std::string& key) {
    try {
        std::string yaml_file_path = ament_index_cpp::get_package_share_directory("au_4d_radar") + "/config/system_info.yaml";        
        YAML::Node config = YAML::LoadFile(yaml_file_path); 

        if (config[key]) {
            return config[key].as<bool>();
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("readPointCloud2Setting"), "not found in system_info.yaml key: %s", key.c_str());    
            return false; 
        }
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("readPointCloud2Setting"), "Error reading YAML file: %s", e.what());  
        return false; 
    }
}

std::string YamlParser::readFrameId(const std::string& key) {
    try {
        std::string yaml_file_path = ament_index_cpp::get_package_share_directory("au_4d_radar") + "/config/system_info.yaml";        
        YAML::Node config = YAML::LoadFile(yaml_file_path); 

        if (config["radars"]) {
            YAML::Node radars = config["radars"];
            
            if (radars[key]) {
                return radars[key].as<std::string>();
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("readFrameId"), "frame_id not found in 'radars' section: %s", key.c_str());
                return "";
            }
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("readFrameId"), "'radars' section not found in system_info.yaml");
            return "";
        }
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("readFrameId"), "Error reading YAML file: %s", e.what());          
        return "";
    }
}

