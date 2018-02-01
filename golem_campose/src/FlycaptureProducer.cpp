#include <golem_campose/FlycaptureProducer.hpp>

#include <ros/ros.h>
#include <mutex>

FlycaptureProducer::FlycaptureProducer()
    : op::Producer(op::ProducerType::Webcam), res(0, 0) {
    fc2::Error error;

    // Initialize camera
    error = this->cam.Connect();
    if(error != fc2::PGRERROR_OK)
        ROS_ERROR("Failed to connect to camera");

    error = this->cam.GetCameraInfo( &this->camInfo );
    if ( error != fc2::PGRERROR_OK )
        ROS_ERROR("Failed to get camera info from camera");

    ROS_INFO("%s %s %u", this->camInfo.vendorName, this->camInfo.modelName, this->camInfo.serialNumber);
    this->is_connected = this->cam.IsConnected();
}

std::string FlycaptureProducer::getFrameName() {
    return std::string("#") + std::to_string(this->frame_num);
}

bool FlycaptureProducer::isOpened() const {
    return this->is_connected;
}

double FlycaptureProducer::get(const int capProperty) {
    if (capProperty == CV_CAP_PROP_FRAME_WIDTH)
    {
        if (get(op::ProducerProperty::Rotation) == 0. || get(op::ProducerProperty::Rotation) == 180.)
            return this->res.x;
        else
            return this->res.y;
    }
    else if (capProperty == CV_CAP_PROP_FRAME_HEIGHT)
    {
        if (get(op::ProducerProperty::Rotation) == 0. || get(op::ProducerProperty::Rotation) == 180.)
            return this->res.y;
        else
            return this->res.x;
    }
    else if (capProperty == CV_CAP_PROP_POS_FRAMES)
        return (double)this->frame_num;
    //else if (capProperty == CV_CAP_PROP_FRAME_COUNT)
    //    return (double)mFilePaths.size();
    else if (capProperty == CV_CAP_PROP_FPS)
        return -1.;
    else
    {
        op::log("Unknown property", op::Priority::Max, __LINE__, __FUNCTION__, __FILE__);
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
        op::log("This property is read-only.", op::Priority::Max, __LINE__, __FUNCTION__, __FILE__);
    else
        op::log("Unknown property", op::Priority::Max, __LINE__, __FUNCTION__, __FILE__);
}

double FlycaptureProducer::get(const op::ProducerProperty property) {
    return op::Producer::get(property);
}

void FlycaptureProducer::set(const op::ProducerProperty property, const double value) {
    op::Producer::set(property, value);
}

cv::Mat FlycaptureProducer::getRawFrame() {
    this->is_connected = this->cam.IsConnected();
    this->frame_num++;

    fc2::Image rawImage, rgbImage;

    fc2::Error error = this->cam.RetrieveBuffer(&rawImage);
    if (error != fc2::PGRERROR_OK) {
            ROS_WARN("Capture error");
            return cv::Mat();
    }

    rawImage.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage);
    unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize()/(double)rgbImage.GetRows();       

    cv::Mat buffer = cv::Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(),rowBytes);
    res.x = buffer.cols;
    res.y = buffer.rows;

    ROS_INFO("Returning frame of size: (%d, %d)", buffer.cols, buffer.rows);
    
    return buffer;
}

void FlycaptureProducer::release() {
    this->cam.StopCapture();
    if(this->cam.IsConnected())
        this->cam.Disconnect();
}

FlycaptureProducer::~FlycaptureProducer() {
    this->release();
}