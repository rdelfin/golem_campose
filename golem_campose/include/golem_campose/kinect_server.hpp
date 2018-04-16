#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <unordered_map>

#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef std::function<bool(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr)> PointCloudCallback;

class KinectServer {
public:
    KinectServer(uint32_t port, PointCloudCallback callback);

    bool start_server();
    bool is_open();

    ~KinectServer();
private:
    void run();

    void client_recv(int sockfd, uint64_t id);
    
    uint32_t port;
    PointCloudCallback callback;
    bool open;

    std::mutex start_mutex;
    std::condition_variable start_cv;
    bool start_done, start_error;
    std::thread* server_thread;

    uint64_t thread_id_count;
    std::unordered_map<uint64_t, std::thread*> socket_threads;
};