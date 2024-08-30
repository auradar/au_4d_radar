/**
 * @file util.hpp
 * @author Antonio Ko(antonioko@au-sensor.com)
 * @brief 
 * @version 0.1
 * @date 2024-08-05
 * 
 * @copyright Copyright AU (c) 2024
 * 
 */

#ifndef UTIL_HPP
#define UTIL_HPP

class Util {

public:
    static std::string readHostnameFromYaml(const std::string& key);
    static std::string readFrameIdFromYaml(const std::string& key);

};

#endif /* UTIL_HPP */
