#include <ros/ros.h>

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "golem_campose_node");

    ros::NodeHandle nh;
    ros::Rate r(10);

    while(ros::ok()) {
        ROS_INFO_STREAM("Hello, world!");
        r.sleep();
        ros::spinOnce();
    }
}