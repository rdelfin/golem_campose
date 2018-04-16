#pragma once

#include <cstdint>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PcSerializer {
public:
    pcl::PointCloud<pcl::PointXYZRGBA> deserialize(uint8_t* data, size_t len);
    pcl::PointCloud<pcl::PointXYZRGBA> deserialize(std::vector<uint8_t> data);
};