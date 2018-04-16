#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <golem_campose/kinect_server.hpp>

bool pc_callback(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr) {
    ROS_INFO("CALLBACK CALLED");
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "kinect_server");

    KinectServer server(5525, pc_callback);
}