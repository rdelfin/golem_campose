#include <ros/ros.h>

#include <golem_campose/FramePoses.h>

#include <std_msgs/Float64MultiArray.h>

ros::Publisher person_publisher;

void keypointCallback(const golem_campose::FramePoses::ConstPtr& msg) {
    ROS_INFO("Message recieved: Frame #%d with %u people.", msg->frame, msg->poses.size());

    if(msg->poses.size() > 0) {
        ROS_INFO("Person 0 info:");
        golem_campose::PersonPose pose = msg->poses[0];

        for(golem_campose::Keypoint kp : pose.keypoint_data) {
            ROS_INFO("\t%.3f %.3f error: %.3f", kp.x, kp.y, kp.confidence);
        }
    }
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "person_finder");
    ros::NodeHandle nh;

    person_publisher = nh.advertise<std_msgs::Float64MultiArray>("people_angles", 1000);
    ros::Subscriber keypoint_subscriber = nh.subscribe("person_keypoints", 1000, keypointCallback);

    ros::spin();
}