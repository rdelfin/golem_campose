#include <mutex>
#include <queue>
#include <unordered_map>

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>

#include <campose_msgs/FramePoses.h>
#include <campose_msgs/PersonAngles.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/ColorRGBA.h>
#include <sensor_msgs/Image.h>

#include <opencv/cv.hpp>

#define CONFIDENCE_THRESHOLD (0.1)
#define POS_TO_THETA         (1)
#define QUEUE_SIZE           (1000)
#define AVG_RADIUS           (5)

std::mutex image_map_lock;
std::unordered_map<int, sensor_msgs::Image> image_map;
std::queue<int> image_queue;

uint64_t rostime_to_long(const ros::Time& time) {
    return ((uint64_t)time.sec)*1000000 + (uint64_t)time.nsec;
}

ros::Publisher person_publisher;

double pose_to_angle(const campose_msgs::PersonPose& person_pose);
bool get_color(std_msgs::ColorRGBA& color, const campose_msgs::Keypoint& torso, std_msgs::Header header);

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
        people_angles.people[idx].color_found = false;

        if(person_pose.neck.confidence > 0 && (person_pose.left_hip.confidence > 0 || person_pose.right_hip.confidence > 0)) {
            // Assume person is vertical
            campose_msgs::Keypoint good_hip = person_pose.left_hip.confidence > 0 ? person_pose.left_hip : person_pose.right_hip;
            campose_msgs::Keypoint torso_pose;
            torso_pose.x = person_pose.neck.x;
            torso_pose.y = person_pose.neck.y + (person_pose.neck.y - good_hip.y) / 2;
            torso_pose.confidence = std::min(person_pose.neck.confidence, good_hip.confidence);
            


            people_angles.people[idx].color_found = get_color(people_angles.people[idx].color, torso_pose, msg->header);
        }
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

bool get_color(std_msgs::ColorRGBA& color, const campose_msgs::Keypoint& torso, std_msgs::Header header) {
    image_map_lock.lock();

    if(image_map.count(rostime_to_long(header.stamp)) == 0) {
        image_map_lock.unlock();
        return false;
    }

    sensor_msgs::Image img_msg = image_map[rostime_to_long(header.stamp)];
    image_map_lock.unlock();

    cv::Mat img = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8)->image.clone();

    double sum_r = 0.0, sum_g = 0.0, sum_b = 0.0;
    int count = 0;
    for(int x = std::max((int)torso.x - AVG_RADIUS, 0); x < std::min((int)torso.x + AVG_RADIUS, img.cols-1); x++) {
        for(int y = std::max((int)torso.y - AVG_RADIUS, 0); y < std::min((int)torso.y + AVG_RADIUS, img.rows-1); y++) {
            cv::Vec3b bgrPixel = img.at<cv::Vec3b>(y, x);
            sum_b += bgrPixel.val[0];
            sum_g += bgrPixel.val[1];
            sum_r += bgrPixel.val[2];
            count++;
        }
    }

    color.r = sum_r / count;
    color.g = sum_g / count;
    color.b = sum_b / count;
    color.a = 1.0;

    return true;
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