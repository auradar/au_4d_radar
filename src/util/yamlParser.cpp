
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

std::unordered_map<uint32_t, RadarInfo> YamlParser::radarsMap_;
std::recursive_mutex YamlParser::radar_map_mutex_;

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

uint32_t YamlParser::readMessageNumber(const std::string& key) {
    try {
        std::string yaml_file_path = ament_index_cpp::get_package_share_directory("au_4d_radar") + "/config/system_info.yaml";
        YAML::Node config = YAML::LoadFile(yaml_file_path);

        if (config[key]) {
            return config[key].as<uint32_t>();
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("readMessageNumber"), "Key '%s' not found in system_info.yaml", key.c_str());
            return 1;
        }
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("readMessageNumber"), "Error reading YAML file: %s", e.what());
        return 1;
    }
}

std::string YamlParser::readFrameId(const std::string& key) {
    try {
        std::string yaml_file_path = ament_index_cpp::get_package_share_directory("au_4d_radar") + "/config/system_info.yaml";
        YAML::Node config = YAML::LoadFile(yaml_file_path);

        if (config["radars"]) {
            YAML::Node radars = config["radars"];

            if (radars[key]) {
                if (radars[key]["frame_id"]) {
                    return radars[key]["frame_id"].as<std::string>();
                } else {
                    RCLCPP_ERROR(rclcpp::get_logger("readFrameId"), "'frame_id' not found for radar: %s", key.c_str());
                    return "";
                }
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("readFrameId"), "Radar not found in 'radars' section: %s", key.c_str());
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

std::unordered_map<uint32_t, RadarInfo> YamlParser::readRadarsAsMap() {
    std::unordered_map<uint32_t, RadarInfo> radars_map;

    try {
        std::string yaml_file_path = ament_index_cpp::get_package_share_directory("au_4d_radar") + "/config/system_info.yaml";
        RCLCPP_INFO(rclcpp::get_logger("readRadarsAsMap"), "Loading YAML file from: %s", yaml_file_path.c_str());
        YAML::Node config = YAML::LoadFile(yaml_file_path);

        if (config["radars"]) {
            YAML::Node radars = config["radars"];

            for (YAML::const_iterator it = radars.begin(); it != radars.end(); ++it) {
                std::string key = it->first.as<std::string>();

                std::stringstream ss;
                uint32_t radar_id;
                ss << std::hex << key;
                ss >> radar_id;

                if (ss.fail()) {
                    RCLCPP_ERROR(rclcpp::get_logger("readRadarsAsMap"), "Failed to convert radar ID: %s", key.c_str());
                    continue;
                }

                RadarInfo radar_info;

                if (it->second["frame_id"]) {
                    radar_info.frame_id = it->second["frame_id"].as<std::string>();
                } else {
                    radar_info.frame_id = "";
                    RCLCPP_ERROR(rclcpp::get_logger("readRadarsAsMap"), "'frame_id' field not found for radar: %s", key.c_str());
                    continue;
                }

                if (it->second["xyz"] && it->second["rpy"]) {
                    YAML::Node xyz = it->second["xyz"];
                    if (xyz.size() == 3) {
                        radar_info.x = xyz[0].as<float>();
                        radar_info.y = xyz[1].as<float>();
                        radar_info.z = xyz[2].as<float>();
                    } else {
                        RCLCPP_ERROR(rclcpp::get_logger("readRadarsAsMap"), "Invalid size for 'xyz' array for radar: %s", key.c_str());
                        continue;
                    }

                    YAML::Node rpy = it->second["rpy"];
                    if (rpy.size() == 3) {
                        radar_info.roll = rpy[0].as<float>();
                        radar_info.pitch = rpy[1].as<float>();
                        radar_info.yaw = rpy[2].as<float>();
                    } else {
                        RCLCPP_ERROR(rclcpp::get_logger("readRadarsAsMap"), "Invalid size for 'rpy' array for radar: %s", key.c_str());
                        continue;
                    }
                } else {
                    RCLCPP_ERROR(rclcpp::get_logger("readRadarsAsMap"), "'xyz' or 'rpy' field not found for radar: %s", key.c_str());
                    continue;
                }

                radars_map[radar_id] = radar_info;
            }
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("readRadarsAsMap"), "'radars' section not found in system_info.yaml");
        }
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("readRadarsAsMap"), "Error reading YAML file: %s", e.what());
    }

    return radars_map;
}

void YamlParser::init() {
    radarsMap_ = readRadarsAsMap();
}

std::string YamlParser::getFrameIdName(uint32_t radar_id) {
    std::lock_guard<std::recursive_mutex> lock(radar_map_mutex_);
    auto it = radarsMap_.find(radar_id);
    if (it != radarsMap_.end()) {
        return it->second.frame_id;
    } else {
        return "";
    }
}

bool YamlParser::checkValidFrameId(uint32_t radar_id) {
    std::lock_guard<std::recursive_mutex> lock(radar_map_mutex_);
    auto it = radarsMap_.find(radar_id);
    if (it != radarsMap_.end()) {
        return true;
    } else {
        return false;
    }
}

RadarInfo YamlParser::getRadarInfo(uint32_t radar_id) {
    std::lock_guard<std::recursive_mutex> lock(radar_map_mutex_);
    auto it = radarsMap_.find(radar_id);
    if (it != radarsMap_.end()) {
        return it->second;
    } else {
        RCLCPP_WARN(rclcpp::get_logger("getRadarInfo"), "Radar with radar_id: %u not found", radar_id);
        return RadarInfo();
    }
}

RadarInfo YamlParser::getRadarInfo(const std::string& frame_id) {
    std::lock_guard<std::recursive_mutex> lock(radar_map_mutex_);
    for (const auto& radar : radarsMap_) {
        if (radar.second.frame_id == frame_id) {
            return radar.second;
        }
    }

    RCLCPP_WARN(rclcpp::get_logger("getRadarInfo"), "Radar with frame_id: %s not found", frame_id.c_str());
    return RadarInfo();
}

void YamlParser::setRadarInfo(const std::string& frame_id, const RadarInfo& radar_info) {
    std::lock_guard<std::recursive_mutex> lock(radar_map_mutex_);
    for (auto& radar : radarsMap_) {
        if (radar.second.frame_id == frame_id) {
            radar.second = radar_info;
            return;
        }
    }
    RCLCPP_WARN(rclcpp::get_logger("setRadarInfo"), "Radar with frame_id: %s not found", frame_id.c_str());
}

