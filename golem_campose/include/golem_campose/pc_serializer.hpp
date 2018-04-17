#pragma once

#include <cstdint>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PcSerializer {
public:
    bool header_size(uint8_t* header_data, uint64_t len, uint64_t& size);
    bool kinect_frame_size(uint8_t* header_data, uint64_t len, uint64_t& frame_size);
    pcl::PointCloud<pcl::PointXYZRGBA> deserialize(uint8_t* data, size_t len);
    pcl::PointCloud<pcl::PointXYZRGBA> deserialize(std::vector<uint8_t> data);
};