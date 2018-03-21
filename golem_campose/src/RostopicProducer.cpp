#include <golem_campose/RostopicProducer.hpp>
#include <cv_bridge/cv_bridge.h>

RostopicProducer::RostopicProducer(std::string topic_name, ros::NodeHandle nh, int queue_size)
    : topic_name(topic_name), Producer(op::ProducerType::Webcam) {
    ROS_INFO("Subscribing to %s", this->topic_name.c_str());
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

    std::unique_lock<std::mutex> lk(this->frame_mtx);

    this->latest_header = msg->header;
    this->latest = cv_ptr->image.clone();
    this->frame_count++;
    this->img_set = true;

    lk.unlock();
    this->frame_cv.notify_all();
}

cv::Mat RostopicProducer::getRawFrame() {
    std::unique_lock<std::mutex> lk(this->frame_mtx);
    this->img_set = false;
    this->frame_cv.wait(lk, [this]{return this->img_set;});

    this->fetched = this->latest.clone();
    this->fetched_header = this->latest_header;
    return this->fetched.clone();
}

std::string RostopicProducer::getFrameName() {
    std::unique_lock<std::mutex> lk(this->frame_mtx);
    int frame_count = this->frame_count;
    return std::string("#") + std::to_string(frame_count);
}

std_msgs::Header RostopicProducer::get_header() {
    std::unique_lock<std::mutex> lk(this->frame_mtx);
    return this->fetched_header;
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
