
/**
 * @file yamlParser.cpp
 * @author Antonio Ko(antonioko@au-sensor.com)
 * @brief 
 * @version 1.0
 * @date 2024-09-11
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

std::unordered_map<uint32_t, std::string> YamlParser::readRadarsAsMap() {
    std::unordered_map<uint32_t, std::string> radars_map;

    try {
        std::string yaml_file_path = ament_index_cpp::get_package_share_directory("au_4d_radar") + "/config/system_info.yaml";
        YAML::Node  config         = YAML::LoadFile(yaml_file_path);

        if (config["radars"]) {
            YAML::Node radars = config["radars"];
            
            for (YAML::const_iterator it = radars.begin(); it != radars.end(); ++it) {
                std::string key   = it->first.as<std::string>();
                std::string value = it->second.as<std::string>();

                std::stringstream ss;
                uint32_t radar_id;
                ss << std::hex << key;                 
                ss >> radar_id;

                if (ss.fail()) {
                    RCLCPP_ERROR(rclcpp::get_logger("readRadarsAsMap"), "Failed to convert radar ID: %s", key.c_str());
                    continue; 
                }
                
                radars_map[radar_id] = value;
            }
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("readRadarsAsMap"), "'radars' section not found in system_info.yaml");
        }
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("readRadarsAsMap"), "Error reading YAML file: %s", e.what());
    }

    return radars_map;
}