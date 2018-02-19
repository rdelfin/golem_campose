#include <mutex>
#include <queue>
#include <unordered_map>

#include <ros/ros.h>

#include <campose_msgs/FramePoses.h>
#include <campose_msgs/PersonAngles.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/ColorRGBA.h>
#include <sensor_msgs/Image.h>

#define CONFIDENCE_THRESHOLD (0.1)
#define POS_TO_THETA         (1)
#define QUEUE_SIZE           (1000)

std::mutex image_map_lock;
std::unordered_map<int, sensor_msgs::Image> image_map;
std::queue<int> image_queue;

uint64_t rostime_to_long(const ros::Time& time) {
    return ((uint64_t)time.sec)*1000000 + (uint64_t)time.nsec;
}

ros::Publisher person_publisher;

double pose_to_angle(const campose_msgs::PersonPose& person_pose);
bool get_color(const std_msgs::ColorRGBA& color, campose_msgs::Keypoint torso);

void image_cb(const sensor_msgs::Image::ConstPtr& msg) {
    uint64_t key;

    image_map_lock.lock();

    if(image_queue.size() != image_map.size())
        ROS_WARN("Person finder image queue and map are not the same size. Map has %lu elements and que has %lu elements.", image_map.size(), image_queue.size());

    while(image_queue.size() > QUEUE_SIZE) {
        key = image_queue.front();
        image_map.erase(key);
        image_queue.pop();
    }

    key = rostime_to_long(msg->header.stamp);
    image_queue.push(key);
    image_map[key] = *msg;

    image_map_lock.unlock();
}

void keypointCallback(const campose_msgs::FramePoses::ConstPtr& msg) {
    campose_msgs::PersonAngles people_angles;
    
    people_angles.header = msg->header;

    for(const campose_msgs::PersonPose& person_pose : msg->poses) {
        people_angles.people.emplace_back();
        size_t idx = people_angles.people.size() - 1;
        people_angles.people[idx].angle = pose_to_angle(person_pose);
    }

    person_publisher.publish(people_angles);
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "person_finder");
    ros::NodeHandle nh;

    std::string camera_topic;
    nh.param<std::string>("camera_topic", camera_topic, "/flycap_cam/image");

    person_publisher = nh.advertise<campose_msgs::PersonAngles>("people_angles", 1000);
    ros::Subscriber keypoint_subscriber = nh.subscribe("person_keypoints", 1000, keypointCallback);
    ros::Subscriber img_subscriber = nh.subscribe(camera_topic, 1000, image_cb);

    ROS_INFO("Converting keypoints to angles...");

    ros::spin();
}

bool get_color(const std_msgs::ColorRGBA& color, campose_msgs::Keypoint torso) {

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