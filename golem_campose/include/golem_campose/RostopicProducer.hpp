#pragma once

#include <ros/ros.h>
#include <openpose/headers.hpp>

#include <sensor_msgs/Image.h>

#include <mutex>
#include <string>

class RostopicProducer : public op::Producer {
public:
    RostopicProducer(std::string topic_name, ros::NodeHandle nh = ros::NodeHandle(), int queue_size = 100);

    virtual std::string getFrameName();

    virtual bool isOpened() const;

    virtual double get(const int capProperty);
    virtual void set(const int capProperty, const double value);
    virtual double get(const op::ProducerProperty property);
    virtual void set(const op::ProducerProperty property, const double value);
    virtual void release();

    std_msgs::Header get_header();

    virtual ~RostopicProducer();

protected:
    virtual cv::Mat getRawFrame();
private:
    std::string topic_name;
    ros::Subscriber sub;
    cv::Mat latest;
    bool img_set;
    int frame_count;
    std::mutex frame_mtx;

    std_msgs::Header fetched_header, latest_header;

    void topic_cb(const sensor_msgs::Image::ConstPtr& msg);

    DELETE_COPY(RostopicProducer);
};
