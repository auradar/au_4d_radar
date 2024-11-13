#ifndef YAML_PARSER_HPP
#define YAML_PARSER_HPP

#include <string>
#include <unordered_map>
#include <mutex>

struct RadarInfo {
    std::string frame_id;
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
};

class YamlParser {

public:
    static std::string readHostname(const std::string& key);
    static bool readPointCloud2Setting(const std::string& key);
    static uint32_t readMessageNumber(const std::string& key);
    static std::string readFrameId(const std::string& key);
    static std::unordered_map<uint32_t, RadarInfo> readRadarsAsMap();

    static void init();
    static std::string getFrameIdName(uint32_t radar_id);
    static bool checkValidFrameId(uint32_t radar_id);
    static RadarInfo getRadarInfo(uint32_t frame_id);
    static RadarInfo getRadarInfo(const std::string& frame_id);
    static void setRadarInfo(const std::string& frame_id, const RadarInfo& radar_info);

private:
    YamlParser() = default;
    ~YamlParser() = default;
    static std::unordered_map<uint32_t, RadarInfo> radarsMap_;
    static std::recursive_mutex radar_map_mutex_;
};

#endif /* YAML_PARSER_HPP */
