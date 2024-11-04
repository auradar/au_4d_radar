#ifndef MESSAGE_PARSE_HPP
#define MESSAGE_PARSE_HPP

#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include <boost/uuid/uuid.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <radar_msgs/msg/radar_return.hpp>
#include <radar_msgs/msg/radar_scan.hpp>
#include <radar_msgs/msg/radar_track.hpp>
#include <radar_msgs/msg/radar_tracks.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "util/yamlParser.hpp"

namespace au_4d_radar
{
class MessageParser
{
public:
    MessageParser()  = default;
    ~MessageParser() = default;

    void parsePointCloud2Msg(uint8_t *p_buff, sensor_msgs::msg::PointCloud2& radar_cloud_msg, bool& complete);
    void parseRadarScanMsg(uint8_t *p_buff, radar_msgs::msg::RadarScan& radar_scan_msg, bool& complete);
    void parseRadarTrackMsg(uint8_t *p_buff, radar_msgs::msg::RadarTracks& radar_tracks_msg, bool& complete);

private:
    void makeRadarPointCloud2Msg(uint8_t *p_buff, sensor_msgs::msg::PointCloud2& cloud_msg, bool& complete);
    void makeRadarScanMsg(uint8_t *p_buff, radar_msgs::msg::RadarScan& radar_scan_msg, bool& complete);
    void makeRadarTracksMsg(uint8_t *p_buff, radar_msgs::msg::RadarTracks& radar_tracks_msg, bool& complete);

    uint32_t sequence_id_;
    std::string frame_id_;
    uint32_t stamp_tv_sec_;
    uint32_t stamp_tv_nsec_;
    std::mutex mtx_point_cloud2;
    std::mutex mtx_radar_scan;
    std::mutex mtx_radar_track;
};

} // namespace au_4d_radar

#endif // MESSAGE_PARSE_HPP
