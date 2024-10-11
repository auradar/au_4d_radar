
#ifndef YAML_PARSER_HPP
#define YAML_PARSER_HPP

#include <string>
#include <unordered_map>

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
    static std::string readFrameId(const std::string& key);
    static std::unordered_map<uint32_t, RadarInfo> readRadarsAsMap();

};

#endif /* YAML_PARSER_HPP */
