#include <golem_campose/FlycaptureProducer.hpp>

#include <ros/ros.h>

FlycaptureProducer::FlycaptureProducer()
    : op::Producer(op::ProducerType::Webcam) {
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
    // Ignore for now. Return 0.
    ROS_WARN("FlycaptureProducer get called (CapProperty). Ignored.");
    return 0;
}

void FlycaptureProducer::set(const int capProperty, const double value) {
    // Ignore for now. Do nothing
    ROS_WARN("FlycaptureProducer set called (capProperty). Ignored.");
}

double FlycaptureProducer::get(const op::ProducerProperty property) {
    // Ignore for now. Return 0.
    ROS_WARN("FlycaptureProducer get called (ProducerProperty). Ignored.");
    return 0;
}

void FlycaptureProducer::set(const op::ProducerProperty property, const double value) {
    // Ignore for now. Do nothing
    ROS_WARN("FlycaptureProducer set called (ProducerProperty). Ignored.");
}

cv::Mat FlycaptureProducer::getRawFrame() {
    this->is_connected = cam.IsConnected();

    fc2::Image rawImage, rgbImage;
    fc2::Error error = cam.RetrieveBuffer( &rawImage );

    if (error != fc2::PGRERROR_OK) {
        ROS_WARN("Capture error with camera on frame %ld", frame_num);
        frame_num++;
        return cv::Mat();
    }

    rawImage.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage );
    unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize()/(double)rgbImage.GetRows();       
    cv::Mat image = cv::Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(),rowBytes);

    frame_num++;
    return image;
}

void FlycaptureProducer::release() {
    if(this->cam.IsConnected())
        this->cam.Disconnect();
}

FlycaptureProducer::~FlycaptureProducer() {
    this->release();
}