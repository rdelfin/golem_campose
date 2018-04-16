#include <golem_campose/pc_serializer.hpp>

#include <sys/types.h>
#include <netinet/in.h>
#include <inttypes.h>
#include <endian.h>

pcl::PointCloud<pcl::PointXYZRGBA> PcSerializer::deserialize(std::vector<uint8_t> data) {
    this->deserialize(&data[0], data.size());
}

pcl::PointCloud<pcl::PointXYZRGBA> PcSerializer::deserialize(uint8_t* data, size_t len) {
    uint8_t* curr = data;
    uint64_t frame_id_len, point_count;
    char* buffer;
    pcl::PointCloud<pcl::PointXYZRGBA> pc;

    pc.header.seq = ntohl(*(uint32_t*)curr);
    curr += sizeof(uint32_t);
    pc.header.stamp = be64toh(*(uint64_t*)curr);
    curr += sizeof(uint64_t);

    // Copy frame (read in length and then string)
    frame_id_len = be64toh(*(uint64_t*)curr);
    curr += sizeof(uint64_t);
    buffer = (char*)malloc(frame_id_len + 1);
    memcpy(buffer, curr, frame_id_len);
    curr += frame_id_len;
    buffer[frame_id_len] = '\0';
    pc.header.frame_id = std::string(buffer);
    free(buffer);

    pc.width = ntohl(*(uint32_t*)curr);
    curr += sizeof(uint32_t);
    pc.height = ntohl(*(uint32_t*)curr);
    curr += sizeof(uint32_t);
    pc.is_dense = (bool)*curr;
    curr++;
    point_count = be64toh(*(uint64_t*)curr);
    curr += sizeof(uint64_t);

    for(uint64_t i = 0; i < point_count; i++) {
        pcl::PointXYZRGBA point;
        point.x = *(float*)&ntohl(*(uint32_t*)curr);
        curr += sizeof(uint32_t);
        point.y = *(float*)&ntohl(*(uint32_t*)curr);
        curr += sizeof(uint32_t);
        point.z = *(float*)&ntohl(*(uint32_t*)curr);
        curr += sizeof(uint32_t);
        point.r = *curr;
        curr += sizeof(uint8_t);
        point.g = *curr;
        curr += sizeof(uint8_t);
        point.b = *curr;
        curr += sizeof(uint8_t);
        point.a = *curr;
        curr += sizeof(uint8_t);

        pc.push_back(point);
    }


}
