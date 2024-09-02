
#ifndef UTIL_HPP
#define UTIL_HPP

class Util {

public:
    static std::string readHostnameFromYaml(const std::string& key);
    static std::string readFrameIdFromYaml(const std::string& key);

};

#endif /* UTIL_HPP */
