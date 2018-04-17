#include <golem_campose/pc_serializer.hpp>

#include <sys/types.h>
#include <netinet/in.h>
#include <inttypes.h>
#include <endian.h>

#define UINT32_TO_FLOAT(val) ({ \
    uint32_t v = val;           \
    float f = *(float*)(&v);  \
    val;                        \
})

bool PcSerializer::header_size(uint8_t* header_data, uint64_t len, uint64_t& size) {
    if(len <= 29)
        return false;

    // 4 bytes for seq + 8 bytes for stamp
    uint64_t frame_id_len = be64toh(*(uint64_t*)(header_data + 12));

    size = 31 + frame_id_len;
    return true;
}

bool PcSerializer::kinect_frame_size(uint8_t* header_data, uint64_t len, uint64_t& frame_size) {
    uint64_t h_size;
    if(!header_size(header_data, len, h_size)) return false;

    // Check the entire header is contained
    if(len < h_size)
        return false;
    
    uint64_t point_count = be64toh(*(uint64_t*)(header_data + h_size - 8));
    uint64_t point_size = 3*sizeof(float) + 4*sizeof(char);
    frame_size = point_count*point_size + h_size;
    return true;
}


pcl::PointCloud<pcl::PointXYZRGBA> PcSerializer::deserialize(std::vector<uint8_t> data) {
    this->deserialize(&data[0], data.size());
}

pcl::PointCloud<pcl::PointXYZRGBA> PcSerializer::deserialize(uint8_t* data, size_t len) {
    uint8_t* curr = data;
    uint64_t frame_id_len, point_count;
    char* buffer;
    pcl::PointCloud<pcl::PointXYZRGBA> pc;
    pc.clear();

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

        point.x = UINT32_TO_FLOAT(ntohl(*(uint32_t*)curr));
        curr += sizeof(uint32_t);
        point.y = UINT32_TO_FLOAT(ntohl(*(uint32_t*)curr));
        curr += sizeof(uint32_t);
        point.z = UINT32_TO_FLOAT(ntohl(*(uint32_t*)curr));
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
