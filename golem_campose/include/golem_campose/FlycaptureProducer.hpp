#pragma once

#include <flycapture/FlyCapture2.h>
#include <openpose/headers.hpp>

namespace fc2 = FlyCapture2;

class camera_not_found_exception : public std::runtime_error {
public:
    camera_not_found_exception()
        : runtime_error("Failed to connect to the camera."){ }
};

class FlycaptureProducer : public op::Producer {
public:
    FlycaptureProducer();

    virtual std::string getFrameName();

    virtual bool isOpened() const;

    virtual double get(const int capProperty);
    virtual void set(const int capProperty, const double value);
    virtual double get(const op::ProducerProperty property);
    virtual void set(const op::ProducerProperty property, const double value);
    virtual void release();

    virtual ~FlycaptureProducer();

protected:
    virtual cv::Mat getRawFrame();
private:
    fc2::Camera cam;
    fc2::CameraInfo camInfo;
    bool is_connected;

    DELETE_COPY(FlycaptureProducer);
};