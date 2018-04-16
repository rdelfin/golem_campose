#include <thread>
#include <mutex>

#include <golem_campose/kinect_server.hpp>

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
        ROS_INFO("Connected to client. Creating new thread...");
        uint64_t thread_id = this->thread_id_count++;
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

void KinectServer::client_recv(int sockfd, uint64_t id) {
    char* buffer = (char *)malloc(sizeof(char)*256);

    if (sockfd < 0)
        ROS_ERROR("ERROR on accept on socket #%lu", id);
    bzero(buffer,256);

    // Handshake
    int n = read(sockfd,buffer,255);
    if(n < 0)
        ROS_ERROR("ERROR reading from socket #%lu", id);

    if(strlen(buffer) == HANDSHAKE_MSG.length() &&
       strncmp(HANDSHAKE_MSG.c_str(), buffer, HANDSHAKE_MSG.length()) == 0)

    n = write(sockfd, HANDSHAKE_MSG.c_str(), HANDSHAKE_MSG.length());
    if(n < 0)
        ROS_ERROR("ERROR writing to socket #%lu", id);

    ROS_INFO("Successfully finished handshake");

    close(sockfd);
    free(buffer);
}