#include <cstdint>
#include <functional>

#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>

#include <pcl/point_types.h>

typedef std::function<bool, pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr> PointCloudCallback;

class KinectServer {
public:
    KinectServer(uint32_t port, PointCloudCallback callback);

    bool start_server();
    bool is_open();

    ~KinectServer();
private:
    void run();
    
    uint32_t port;
    PointCloudCallback callback;
    bool open;

    std::mutex start_mutex;
    std::condition_variable start_cv;
    bool start_done, start_error;
    std::thread* server_thread;
};