#include <thread>
#include <mutex>

#include <golem_campose/kinect_server.hpp>

#include <ros/ros.h>

KinectServer::KinectServer(uint32_t port, PointCloudCallback callback)
    : port(port), callback(callback), open(false) {

}

bool KinectServer::start_server() {
    this->start_done = this->start_error = false;
    this->server_thread = new std::thread(&KinectServer::run, this);
    std::unique_lock<std::mutex> lock(this->start_mutex);
    this->start_cv.wait(lock, [this]{return this->start_done;});
    return this->start_error;
}

// Code from http://www.cs.rpi.edu/~moorthy/Courses/os98/Pgms/server.c
void KinectServer::run() {
    std::unique_lock<std::mutex> lock(this->start_mutex);

    int sockfd, newsockfd, clilen;
    char buffer[256];
    struct sockaddr_in serv_addr, cli_addr;
    int n;

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

    this->start_error = false;
    this->open = true;
    this->start_done = true;
    lock.unlock();
    this->start_cv.notify_all();


    listen(sockfd,5);
    clilen = sizeof(cli_addr);
    newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, (socklen_t*)&clilen);
    if (newsockfd < 0) 
        ROS_ERROR("ERROR on accept");
    bzero(buffer,256);
    n = read(newsockfd,buffer,255);
    if(n < 0)
        ROS_ERROR("ERROR reading from socket");
    ROS_INFO("Here is the message: %s\n",buffer);

    n = write(newsockfd,"I got your message",18);
    if(n < 0)
        ROS_ERROR("ERROR writing to socket");
}

bool KinectServer::is_open() {
    return this->open;
}

KinectServer::~KinectServer() {
    this->open = false;
}