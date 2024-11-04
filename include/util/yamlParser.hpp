#ifndef YAML_PARSER_HPP
#define YAML_PARSER_HPP

#include <string>
#include <unordered_map>
#include <mutex>

class YamlParser {

public:
    static std::string readHostname(const std::string& key);
    static bool readPointCloud2Setting(const std::string& key);
    static uint32_t readMessageNumber(const std::string& key);
    static std::string readFrameId(const std::string& key);
    static std::unordered_map<uint32_t, std::string> readRadarsAsMap();

    static void init();
    static std::string getFrameId(uint32_t radar_id);
    static bool checkValidFrameId(uint32_t radar_id);

private:
    YamlParser() = default;
    ~YamlParser() = default;

    static std::recursive_mutex radar_map_mutex_;
    static std::unordered_map<uint32_t, std::string> radarsMap_;
};

#endif /* YAML_PARSER_HPP */
