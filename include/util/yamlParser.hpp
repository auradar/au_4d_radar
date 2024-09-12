
#ifndef YAML_PARSER_HPP
#define YAML_PARSER_HPP

class YamlParser {

public:
    static std::string readHostname(const std::string& key);
    static bool readPointCloud2Setting(const std::string& key);    
    static std::string readFrameId(const std::string& key);
    static std::map<uint32_t, std::string> readRadarsAsMap();     

};

#endif /* YAML_PARSER_HPP */
