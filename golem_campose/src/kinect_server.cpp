#include <thread>
#include <mutex>

#include <golem_campose/kinect_server.hpp>
#include <golem_campose/pc_serializer.hpp>

#include <ros/ros.h>

KinectServer::KinectServer(uint32_t port, PointCloudCallback callback)
    : port(port), callback(callback), open(false),
      server_thread(nullptr), thread_id_count(0) {

}

bool KinectServer::start_server() {
    this->start_done = this->start_error = false;
    this->server_thread = new std::thread(&KinectServer::run, this);
    std::unique_lock<std::mutex> lock(this->start_mutex);
    this->start_cv.wait(lock, [this]{return this->start_done;});
    return !this->start_error;
}

// Code from http://www.cs.rpi.edu/~moorthy/Courses/os98/Pgms/server.c
void KinectServer::run() {
    ROS_INFO("Starting the kinect server.");

    std::unique_lock<std::mutex> lock(this->start_mutex);

    int sockfd, client_sockfd, clilen;
    struct sockaddr_in serv_addr, cli_addr;

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if(sockfd < 0) {
        ROS_ERROR("ERROR opening socket");
        this->start_error = true;
        this->start_done = true;
        lock.unlock();
        this->start_cv.notify_all();
        return;
    }
    memset((char *) &serv_addr, 0, sizeof(serv_addr));;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(this->port);
    if(bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)  {
        ROS_ERROR("ERROR on binding");
        this->start_error = true;
        this->start_done = true;
        lock.unlock();
        this->start_cv.notify_all();
        return;
    }

    ROS_INFO("Socket binded.");

    this->start_error = false;
    this->open = true;
    this->start_done = true;
    lock.unlock();
    this->start_cv.notify_all();

    ROS_INFO("Socket listening...");

    listen(sockfd,5);
    clilen = sizeof(cli_addr);
    while(client_sockfd = accept(sockfd, (struct sockaddr *) &cli_addr, (socklen_t*)&clilen)) {
        uint64_t thread_id = this->thread_id_count++;
        ROS_INFO("Connected to client. Creating new thread at id %lu...", thread_id);
        this->socket_threads.insert(
            {thread_id,
            new std::thread(&KinectServer::client_recv, this, client_sockfd, this->thread_id_count-1)});
    }
}

bool KinectServer::is_open() {
    return this->open;
}

KinectServer::~KinectServer() {
    this->open = false;
    if(server_thread != nullptr)
        delete server_thread;

    for(std::pair<const uint64_t, std::thread*>& p : this->socket_threads) {
        delete p.second;
    }
}

#define BUFFER_SIZE 1024

void KinectServer::client_recv(int sockfd, uint64_t id) {
    char* buffer = (char *)malloc(sizeof(char)*BUFFER_SIZE);
    PcSerializer pcs;
    std::vector<uint8_t> v_buffer;
    v_buffer.reserve(BUFFER_SIZE);

    if (sockfd < 0) {
        ROS_ERROR("Socket %lu\tERROR on accept", id);
        close(sockfd);
        free(buffer);
        return;
    }

    bzero(buffer,BUFFER_SIZE);

    // Handshake
    int n = read(sockfd,buffer,BUFFER_SIZE - 1);
    if(n < 0) {
        ROS_ERROR("Socket %lu\tERROR reading handshake", id);
        close(sockfd);
        free(buffer);
        return;
    }

    if(strlen(buffer) == HANDSHAKE_MSG.length() &&
       strncmp(HANDSHAKE_MSG.c_str(), buffer, HANDSHAKE_MSG.length()) == 0)

    n = write(sockfd, HANDSHAKE_MSG.c_str(), HANDSHAKE_MSG.length());
    if(n < 0) {
        ROS_ERROR("Socket %lu\tERROR writing to socket", id);
        close(sockfd);
        free(buffer);
        return;
    }

    ROS_INFO("Socket %lu\tSuccessfully finished handshake", id);

    bool running = true;
    bool parsing = false;

    while(running) {
        memset(buffer, 0, BUFFER_SIZE);
        n = read(sockfd, buffer, BUFFER_SIZE - 1);

        if(n < 0) {
            ROS_ERROR("Socket %lu\tERROR reading socket", id);
            close(sockfd);
            free(buffer);
        } if(n > 0) {
            uint8_t* ptr = &v_buffer[v_buffer.size()];
            v_buffer.resize(v_buffer.size()+n);
            memcpy(ptr, buffer, n);
        }

        if(!parsing) {
            bool header_ok = false;
            while(!header_ok && v_buffer.size() >= 4) {
                if(memcmp(&v_buffer[0], "snd", 3) == 0) {
                    header_ok = true;
                    // Remove header
                    parsing = true;
                    v_buffer.erase(v_buffer.begin(), v_buffer.begin() + 4);

                } else if(memcmp(&v_buffer[0], "end", 3) == 0) {
                    header_ok = true;
                    // You can close the socket now.
                    running = false;
                } else {
                    // Invalid header. Skip to next char
                    v_buffer.erase(v_buffer.begin());
                }
            }
        }

        // value of parsing may change during the last if
        if(parsing) {
            uint64_t msg_size;
            // Get header size
            if(pcs.kinect_frame_size(&v_buffer[4], v_buffer.size(), msg_size) && v_buffer >= msg_size) {
                
            }
        }

    }

    close(sockfd);
    free(buffer);
}