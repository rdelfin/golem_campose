#include <ros/ros.h>

#include <campose_msgs/FramePoses.h>

#include <std_msgs/Float64MultiArray.h>

#define CONFIDENCE_THRESHOLD (0.1)
#define POS_TO_THETA         (1)


ros::Publisher person_publisher;

double pose_to_angle(const campose_msgs::PersonPose& person_pose);

void keypointCallback(const campose_msgs::FramePoses::ConstPtr& msg) {
    std_msgs::Float64MultiArray people_angles;

    for(const campose_msgs::PersonPose& person_pose : msg->poses) {
        people_angles.data.push_back(pose_to_angle(person_pose));
    }

    person_publisher.publish(people_angles);
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "person_finder");
    ros::NodeHandle nh;

    person_publisher = nh.advertise<std_msgs::Float64MultiArray>("people_angles", 1000);
    ros::Subscriber keypoint_subscriber = nh.subscribe("person_keypoints", 1000, keypointCallback);

    ROS_INFO("Converting keypoints to angles...");

    ros::spin();
}

double pose_to_angle(const campose_msgs::PersonPose& person_pose) {
    double pos_x;
    if(person_pose.nose.confidence > CONFIDENCE_THRESHOLD ||
       person_pose.neck.confidence > CONFIDENCE_THRESHOLD) {
        campose_msgs::Keypoint neck = person_pose.neck, nose = person_pose.nose;
        // Set pos_x as a weighted average of the neck and nose x weighted on the confidence
        pos_x = (neck.confidence*neck.x + nose.confidence*nose.x) / (neck.confidence + nose.confidence);
    } else {
        // If the nsoe and neck can't be found, create a bounding box around all keypoints
        // with confidence > CONFIDENCE_THRESHOLD and return the center.
        bool minmax_set = false;
        double xmin = 0, xmax = 0;
        for(campose_msgs::Keypoint kp : person_pose.keypoint_data) {
            if(kp.confidence > CONFIDENCE_THRESHOLD) {
                if(!minmax_set) {
                    xmin = kp.x;
                    xmax = kp.x;
                    minmax_set = true;
                }

                if(kp.x < xmin)
                    xmin = kp.x;
                if(kp.x > xmax)
                    xmax = kp.x;
            }
        }

        pos_x = xmin + (xmax-xmin)/2.0;
    }

    double centered_pos_x = (pos_x - 0.5) * 2.0;

    return centered_pos_x * POS_TO_THETA;
}