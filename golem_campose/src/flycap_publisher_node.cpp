#include <ros/ros.h>

#include <flycapture/FlyCapture2.h>

#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>

namespace fc2 = FlyCapture2;

ros::Publisher img_pub;

int frame_id = 0;

void frame_recv(fc2::Image* pImage, const void* pCallbackData) {
    fc2::Image rgbImage;
    cv_bridge::CvImagePtr cv_ptr;

    pImage->Convert(FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage);
    unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize()/(double)rgbImage.GetRows();
    cv::Mat img(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(),rowBytes);

    sensor_msgs::ImagePtr msg_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    msg_img->header.seq = frame_id;
    msg_img->header.frame_id = std::to_string(frame_id);
    msg_img->header.stamp = ros::Time::now();

    img_pub.publish(msg_img);
    frame_id++;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "flycap_publisher");

    ros::NodeHandle nh;

    img_pub = nh.advertise<sensor_msgs::Image>("flycap_cam/image", 30);

    fc2::Error error;
    fc2::Camera cam;
    fc2::CameraInfo camInfo;

    // Initialize camera
    error = cam.Connect();
    if(error != fc2::PGRERROR_OK) {
        ROS_ERROR("There was an error connecting to flycap camera.");
        return -1;
    }

    error = cam.GetCameraInfo( &camInfo );
    if ( error != fc2::PGRERROR_OK ) {
        ROS_ERROR("There was an error getting the camera information.");
        return -1;
    }

    ROS_INFO("Camera: %s %s %u", camInfo.vendorName, camInfo.modelName, camInfo.serialNumber);
    cam.StartCapture(frame_recv);

    ros::spin();
}