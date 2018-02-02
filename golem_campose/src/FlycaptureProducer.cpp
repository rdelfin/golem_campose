#include <golem_campose/FlycaptureProducer.hpp>

#include <ros/ros.h>
#include <mutex>

std::mutex buffer_mutex;
cv::Mat buffer;
long frame_num = 0;

void frame_recv(fc2::Image* pImage, const void* pCallbackData) {
    fc2::Image rgbImage;

    pImage->Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage );
    unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize()/(double)rgbImage.GetRows();       

    buffer_mutex.lock();
    buffer = cv::Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(),rowBytes);

    frame_num++;
    buffer_mutex.unlock();

    ROS_INFO("FRAME RECIEVED. OF SIZE: (%d, %d)", buffer.cols, buffer.rows);
}

FlycaptureProducer::FlycaptureProducer()
    : op::Producer(op::ProducerType::Webcam) {
    fc2::Error error;

    // Initialize camera
    error = this->cam.Connect();
    if(error != fc2::PGRERROR_OK)
        throw camera_not_found_exception();

    error = this->cam.GetCameraInfo( &this->camInfo );
    if ( error != fc2::PGRERROR_OK )
        throw camera_not_found_exception();

    ROS_INFO("%s %s %u", this->camInfo.vendorName, this->camInfo.modelName, this->camInfo.serialNumber);
    this->is_connected = this->cam.IsConnected();

    this->cam.StartCapture(frame_recv);
}

std::string FlycaptureProducer::getFrameName() {
    buffer_mutex.lock();
    int frame_num_cpy = frame_num;
    buffer_mutex.unlock();
    return std::string("#") + std::to_string(frame_num_cpy);
}

bool FlycaptureProducer::isOpened() const {
    return this->is_connected;
}

double FlycaptureProducer::get(const int capProperty) {
    if (capProperty == CV_CAP_PROP_FRAME_WIDTH)
    {
        return buffer.cols;
        //if (get(ProducerProperty::Rotation) == 0. || get(ProducerProperty::Rotation) == 180.)
        //    return mResolution.x;
        //else
        //    return mResolution.y;
    }
    else if (capProperty == CV_CAP_PROP_FRAME_HEIGHT)
    {
        return buffer.rows;
        //if (get(ProducerProperty::Rotation) == 0. || get(ProducerProperty::Rotation) == 180.)
        //    return mResolution.y;
        //else
        //    return mResolution.x;
    }
    else if (capProperty == CV_CAP_PROP_POS_FRAMES)
        return (double)frame_num;
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

void FlycaptureProducer::set(const int capProperty, const double value) {
    //if (capProperty == CV_CAP_PROP_FRAME_WIDTH)
        //mResolution.x = {(int)value};
    //else if (capProperty == CV_CAP_PROP_FRAME_HEIGHT)
    //    mResolution.y = {(int)value};
    //if (capProperty == CV_CAP_PROP_POS_FRAMES)
    //    mFrameNameCounter = fastTruncate((long long)value, 0ll, (long long)mFilePaths.size()-1);
    if (capProperty == CV_CAP_PROP_FRAME_COUNT || capProperty == CV_CAP_PROP_FPS)
        ROS_WARN("The property (%d) is read only on %s:%s:%d", capProperty, __FILE__, __FUNCTION__, __LINE__);
    else
        ROS_WARN("Unknown property (%d) on %s:%s:%d", capProperty, __FILE__, __FUNCTION__, __LINE__);
}

double FlycaptureProducer::get(const op::ProducerProperty property) {
    return op::Producer::get(property);
}

void FlycaptureProducer::set(const op::ProducerProperty property, const double value) {
    op::Producer::set(property, value);
}

cv::Mat FlycaptureProducer::getRawFrame() {
    this->is_connected = cam.IsConnected();

    buffer_mutex.lock();
    cv::Mat my_frame = buffer.clone();
    buffer_mutex.unlock();

    ROS_INFO("Returning frame of size: (%d, %d)", my_frame.cols, my_frame.rows);
    
    return my_frame;
}

void FlycaptureProducer::release() {
    this->cam.StopCapture();
    if(this->cam.IsConnected())
        this->cam.Disconnect();
}

FlycaptureProducer::~FlycaptureProducer() {
    this->release();
}