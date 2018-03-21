#include <string>

#include <ros/ros.h>

#include <campose_msgs/FramePoses.h>
#include <campose_msgs/PersonPose.h>
#include <campose_msgs/Keypoint.h>

#include <cv_bridge/cv_bridge.h>

#include <openpose/headers.hpp>

#include <opencv2/opencv.hpp>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <sensor_msgs/Image.h>

// Allow Google Flags in Ubuntu 14
#ifndef GFLAGS_GFLAGS_H_
    namespace gflags = google;
#endif

// OpenPose
DEFINE_string(model_pose,               "COCO",         "Model to be used. E.g. `COCO` (18 keypoints), `MPI` (15 keypoints, ~10% faster), "
                                                        "`MPI_4_layers` (15 keypoints, even faster but less accurate).");
DEFINE_string(model_folder,             "/home/golem/Documents/openpose/models/",      "Folder path (absolute or relative) where the models (pose, face, ...) are located.");
// Originally was -1x368
DEFINE_string(net_resolution,           "320x176",      "Multiples of 16. If it is increased, the accuracy potentially increases. If it is"
                                                        " decreased, the speed increases. For maximum speed-accuracy balance, it should keep the"
                                                        " closest aspect ratio possible to the images or videos to be processed. Using `-1` in"
                                                        " any of the dimensions, OP will choose the optimal aspect ratio depending on the user's"
                                                        " input value. E.g. the default `-1x368` is equivalent to `656x368` in 16:9 resolutions,"
                                                        " e.g. full HD (1980x1080) and HD (1280x720) resolutions.");
DEFINE_string(output_resolution,        "-1x-1",        "The image resolution (display and output). Use \"-1x-1\" to force the program to use the"
                                                        " input image resolution.");
DEFINE_int32(num_gpu_start,             0,              "GPU device start number.");
DEFINE_double(scale_gap,                0.3,            "Scale gap between scales. No effect unless scale_number > 1. Initial scale is always 1."
                                                        " If you want to change the initial scale, you actually want to multiply the"
                                                        " `net_resolution` by your desired initial scale.");
DEFINE_int32(scale_number,              1,              "Number of scales to average.");
// OpenPose Rendering
DEFINE_int32(part_to_show,              19,             "Prediction channel to visualize (default: 0). 0 for all the body parts, 1-18 for each body"
                                                        " part heat map, 19 for the background heat map, 20 for all the body part heat maps"
                                                        " together, 21 for all the PAFs, 22-40 for each body part pair PAF");
DEFINE_bool(disable_blending,           false,          "If enabled, it will render the results (keypoint skeletons or heatmaps) on a black"
                                                        " background, instead of being rendered into the original image. Related: `part_to_show`,"
                                                        " `alpha_pose`, and `alpha_pose`.");
DEFINE_double(render_threshold,         0.05,           "Only estimated keypoints whose score confidences are higher than this threshold will be"
                                                        " rendered. Generally, a high threshold (> 0.5) will only render very clear body parts;"
                                                        " while small thresholds (~0.1) will also output guessed and occluded keypoints, but also"
                                                        " more false positives (i.e. wrong detections).");
DEFINE_double(alpha_pose,               0.6,            "Blending factor (range 0-1) for the body part rendering. 1 will show it completely, 0 will"
                                                        " hide it. Only valid for GPU rendering.");
DEFINE_double(alpha_heatmap,            0.7,            "Blending factor (range 0-1) between heatmap and original frame. 1 will only show the"
                                                        " heatmap, 0 will only show the frame. Only valid for GPU rendering.");
DEFINE_int32(render_pose,               -1,             "Set to 0 for no rendering, 1 for CPU rendering (slightly faster), and 2 for GPU rendering"
                                                        " (slower but greater functionality, e.g. `alpha_X` flags). If -1, it will pick CPU if"
                                                        " CPU_ONLY is enabled, or GPU if CUDA is enabled. If rendering is enabled, it will render"
                                                        " both `outputData` and `cvOutputData` with the original image and desired body part to be"
                                                        " shown (i.e. keypoints, heat maps or PAFs).");

DEFINE_int32(img_height,                700,            "The hight in pixels of the image rendered.");

op::Point<int> outputSize;
op::Point<int> netInputSize;
op::Point<int> netOutputSize;
op::PoseModel poseModel;

op::ScaleAndSizeExtractor *scaleAndSizeExtractor;
op::CvMatToOpInput cvMatToOpInput;
op::CvMatToOpOutput cvMatToOpOutput;
op::PoseExtractorCaffe *poseExtractorCaffe;

ros::Publisher person_keypoint_pub;

void fill_pose_with_keypoints(campose_msgs::PersonPose& pose, const op::Array<float>& keypoints, int person) {
    pose.keypoint_data.resize(keypoints.getSize(1));
    for (auto bodyPart = 0; bodyPart < keypoints.getSize(1); bodyPart++) {
        pose.keypoint_data[bodyPart].x          = keypoints[{person, bodyPart, 0}];
        pose.keypoint_data[bodyPart].y          = keypoints[{person, bodyPart, 1}];
        pose.keypoint_data[bodyPart].confidence = keypoints[{person, bodyPart, 2}];
    }

    if(pose.keypoint_data.size() < 18)
        return;

    pose.nose           = pose.keypoint_data[0];
    pose.neck           = pose.keypoint_data[1];
    pose.right_shoulder = pose.keypoint_data[2];
    pose.right_elbow    = pose.keypoint_data[3];
    pose.right_wrist    = pose.keypoint_data[4];
    pose.left_shoulder  = pose.keypoint_data[5];
    pose.left_elbow     = pose.keypoint_data[6];
    pose.left_wrist     = pose.keypoint_data[7];
    pose.right_hip      = pose.keypoint_data[8];
    pose.right_knee     = pose.keypoint_data[9];
    pose.right_ankle    = pose.keypoint_data[10];
    pose.left_hip       = pose.keypoint_data[11];
    pose.left_knee      = pose.keypoint_data[12];
    pose.left_ankle     = pose.keypoint_data[13];
    pose.right_eye      = pose.keypoint_data[14];
    pose.left_eye       = pose.keypoint_data[15];
    pose.right_ear      = pose.keypoint_data[16];
    pose.left_ear       = pose.keypoint_data[17];
}

// Again, reference https://github.com/ildoonet/ros-openpose/blob/master/openpose_ros_node/src/openpose_ros_node.cpp
void camera_cb(const sensor_msgs::Image::ConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception& e) {
        return;
    }
    if (cv_ptr->image.empty()) return;

    const op::Point<int> imageSize{cv_ptr->image.cols, cv_ptr->image.rows};

    std::vector<double> scaleInputToNetInputs;
    std::vector<op::Point<int>> netInputSizes;
    double scaleInputToOutput;
    op::Point<int> outputResolution;

    // process
    std::tie(scaleInputToNetInputs, netInputSizes, scaleInputToOutput, outputResolution)
        = scaleAndSizeExtractor->extract(imageSize);

    const auto netInputArray = cvMatToOpInput.createArray(cv_ptr->image, scaleInputToNetInputs, netInputSizes);
    auto outputArray = cvMatToOpOutput.createArray(cv_ptr->image, scaleInputToOutput, outputResolution);
    // Step 3 - Estimate poseKeypoints
    poseExtractorCaffe->forwardPass(netInputArray, imageSize, scaleInputToNetInputs);
    const auto poseKeypoints = poseExtractorCaffe->getPoseKeypoints();

    // publish skeleton info.
    std_msgs::Header header = msg->header;

    campose_msgs::FramePoses framePosesMsg;
    framePosesMsg.header = header;
    framePosesMsg.poses.resize(poseKeypoints.getSize(0));

    for (auto person = 0; person < poseKeypoints.getSize(0); person++) {
        fill_pose_with_keypoints(framePosesMsg.poses[person], poseKeypoints, person);
    }
    
    person_keypoint_pub.publish(framePosesMsg);
}

// Heavily based on ildoonet's ros-openpose node here: https://github.com/ildoonet/ros-openpose/blob/master/openpose_ros_node/src/openpose_ros_node.cpp
int main(int argc, char* argv[]) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    // ROS initialization
    ros::init(argc, argv, "golem_campose_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    ros::Rate r(10);

    std::string camera_topic;
    nh.param<std::string>("camera_topic", camera_topic, "/flycap_cam/image");

    // Supress OpenPose logging
    FLAGS_logtostderr = false;
    google::InitGoogleLogging(argv[0]);
    op::ConfigureLog::setPriorityThreshold(op::Priority::NoOutput);
    op::Profiler::setDefaultX(1000);

    // outputSize
    outputSize = op::flagsToPoint(FLAGS_output_resolution, "-1x-1");
    // netInputSize
    netInputSize = op::flagsToPoint(FLAGS_net_resolution, "-1x368");
    // poseModel
    poseModel = op::flagsToPoseModel(FLAGS_model_pose);
    // Check no contradictory flags enabled
    if (FLAGS_alpha_pose < 0. || FLAGS_alpha_pose > 1.)
        ROS_ERROR("Alpha value for blending must be in the range [0,1]. Found in %s:%s:%d", __FILE__, __FUNCTION__, __LINE__);
    if (FLAGS_scale_gap <= 0. && FLAGS_scale_number > 1)
        ROS_ERROR("Incompatible flag configuration: scale_gap must be greater than 0 or scale_number = 1. Found in %s:%s:%d",
                  __FILE__, __FUNCTION__, __LINE__);\
    netOutputSize = netInputSize;   

    const auto poseModel = op::flagsToPoseModel(FLAGS_model_pose);
    const auto heatMapTypes = op::flagsToHeatMaps(false, false, false);
    const auto heatMapScale = op::flagsToHeatMapScaleMode(2);



    // Initialize
    scaleAndSizeExtractor = new op::ScaleAndSizeExtractor(netInputSize, outputSize, FLAGS_scale_number, FLAGS_scale_gap);
    poseExtractorCaffe = new op::PoseExtractorCaffe(poseModel, FLAGS_model_folder, FLAGS_num_gpu_start);

    poseExtractorCaffe->initializationOnThread();

    ros::Subscriber image_sub = nh.subscribe(camera_topic, 1, camera_cb);
    person_keypoint_pub = nh.advertise<campose_msgs::FramePoses>("person_keypoints", 1000);
    
    spinner.start();
    ros::waitForShutdown();
}