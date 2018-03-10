#include <golem_campose/RostopicProducer.hpp>
#include <cv_bridge/cv_bridge.h>

RostopicProducer::RostopicProducer(std::string topic_name, ros::NodeHandle nh, int queue_size)
    : topic_name(topic_name), Producer(op::ProducerType::Webcam) {
    this->sub = nh.subscribe(this->topic_name, queue_size, &RostopicProducer::topic_cb, this);
    this->img_set = false;
    this->frame_count = 0;
}

void RostopicProducer::topic_cb(const sensor_msgs::Image::ConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch(cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    this->frame_mtx.lock();
    this->latest_header = msg->header;
    this->latest = cv_ptr->image.clone();
    this->img_set = true;
    this->frame_count++;
    this->frame_mtx.unlock();
}

cv::Mat RostopicProducer::getRawFrame() {
    this->frame_mtx.lock();
    cv::Mat result = this->latest.clone();
    this->fetched_header = this->latest_header;
    this->frame_mtx.unlock();

    return this->latest;
}

std::string RostopicProducer::getFrameName() {
    this->frame_mtx.lock();
    int frame_count = this->frame_count;
    this->frame_mtx.unlock();
    return std::string("#") + std::to_string(frame_count);
}

std_msgs::Header RostopicProducer::get_header() {
    this->frame_mtx.lock();
    std_msgs::Header header = this->fetched_header;
    this->frame_mtx.unlock();
    return header;
}

bool RostopicProducer::isOpened() const {
    return true;
}

double RostopicProducer::get(const int capProperty) {
    if (capProperty == CV_CAP_PROP_FRAME_WIDTH)
        return this->latest.cols;
    else if (capProperty == CV_CAP_PROP_FRAME_HEIGHT)
        return this->latest.rows;
    else if (capProperty == CV_CAP_PROP_POS_FRAMES)
        return (double)this->frame_count;
    else if (capProperty == CV_CAP_PROP_FRAME_COUNT)
        return -1;
    else if (capProperty == CV_CAP_PROP_FPS)
        return -1.;
    else
    {
        ROS_WARN("Unknown property (%d) on %s:%s:%d", capProperty, __FILE__, __FUNCTION__, __LINE__);
        return -1.;
    }
}

void RostopicProducer::set(const int capProperty, const double value) {
    if (capProperty == CV_CAP_PROP_FRAME_COUNT || capProperty == CV_CAP_PROP_FPS)
        ROS_WARN("The property (%d) is read only on %s:%s:%d", capProperty, __FILE__, __FUNCTION__, __LINE__);
    else
        ROS_WARN("Unknown property (%d) on %s:%s:%d", capProperty, __FILE__, __FUNCTION__, __LINE__);
}

double RostopicProducer::get(const op::ProducerProperty property) {
    return op::Producer::get(property);
}

void RostopicProducer::set(const op::ProducerProperty property, const double value) {
    op::Producer::set(property, value);
}

void RostopicProducer::release() { }

RostopicProducer::~RostopicProducer() { }
