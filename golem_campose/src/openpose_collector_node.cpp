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

const std::string WINDOW_NAME = "Window 1";

std::shared_ptr<op::PoseExtractorCaffe> poseExtractorPtr;
op::ScaleAndSizeExtractor *scaleAndSizeExtractor;
op::CvMatToOpInput cvMatToOpInput;
op::CvMatToOpOutput cvMatToOpOutput;
op::OpOutputToCvMat opOutputToCvMat;

ros::Publisher person_keypoint_pub;
op::PoseGpuRenderer* poseGpuRenderer;

std::vector<op::Array<float>> netInputArray;
op::Array<float> outputArray;

int IMAGE_WIDTH;

void display_person_image(const cv::Mat& img_mat, const campose_msgs::FramePoses& framePosesMsg) {
    cv::Mat edited_img = img_mat.clone();

    // Iterate over every person
    for(campose_msgs::PersonPose person : framePosesMsg.poses) {
        // First, add each joint point
        for(campose_msgs::Keypoint kp : person.keypoint_data)
            if(kp.confidence > 0)
                cv::circle(edited_img, cv::Point((int)kp.x, (int)kp.y), 20, cv::Scalar(255, 0, 0), -1);

        // Next, add every "bone" in the skeleton
        cv::Point neck          (person.neck.x,           person.neck.y),
                  nose          (person.nose.x,           person.nose.y),
                  right_shoulder(person.right_shoulder.x, person.right_shoulder.y),
                  right_elbow   (person.right_elbow.x,    person.right_elbow.y),
                  right_wrist   (person.right_wrist.x,    person.right_wrist.y),
                  left_shoulder (person.left_shoulder.x,  person.left_shoulder.y),
                  left_elbow    (person.left_elbow.x,     person.left_elbow.y),
                  left_wrist    (person.left_wrist.x,     person.left_wrist.y),
                  right_hip     (person.right_hip.x,      person.right_hip.y),
                  right_knee    (person.right_knee.x,     person.right_knee.y),
                  right_ankle   (person.right_ankle.x,    person.right_ankle.y),
                  left_hip      (person.left_hip.x,       person.left_hip.y),
                  left_knee     (person.left_knee.x,      person.left_knee.y),
                  left_ankle    (person.left_ankle.x,     person.left_ankle.y),
                  right_eye     (person.right_eye.x,      person.right_eye.y),
                  left_eye      (person.left_eye.x,       person.left_eye.y),
                  right_ear     (person.right_ear.x,      person.right_ear.y),
                  left_ear      (person.left_ear.x,       person.left_ear.y);
        cv::Scalar line_color(0, 0, 255);
        int line_thickness = 10;


        // Nose-neck
        if(person.nose.confidence > 0 && person.neck.confidence > 0)
            cv::line(edited_img, nose, neck, line_color, line_thickness);
        // rshoulder-neck
        if(person.right_shoulder.confidence > 0 && person.neck.confidence > 0)
            cv::line(edited_img, right_shoulder, neck, line_color, line_thickness);
        // rshoulder-relbow
        if(person.right_elbow.confidence > 0 && person.right_shoulder.confidence > 0)
            cv::line(edited_img, right_elbow, right_shoulder, line_color, line_thickness);
        // rwrist-relbow
        if(person.right_elbow.confidence > 0 && person.right_wrist.confidence > 0)
            cv::line(edited_img, right_elbow, right_wrist, line_color, line_thickness);
        // lshoulder-neck
        if(person.left_shoulder.confidence > 0 && person.neck.confidence > 0)
            cv::line(edited_img, left_shoulder, neck, line_color, line_thickness);
        // lelbow-lshoulder
        if(person.left_elbow.confidence > 0 && person.left_shoulder.confidence > 0)
            cv::line(edited_img, left_elbow, left_shoulder, line_color, line_thickness);
        // lwrist-lelbow
        if(person.left_elbow.confidence > 0 && person.left_wrist.confidence > 0)
            cv::line(edited_img, left_elbow, left_wrist, line_color, line_thickness);
        // rhip-neck
        if(person.right_hip.confidence > 0 && person.neck.confidence > 0)
            cv::line(edited_img, right_hip, neck, line_color, line_thickness);
        // rknee-rhip
        if(person.right_hip.confidence > 0 && person.right_knee.confidence > 0)
            cv::line(edited_img, right_knee, right_hip, line_color, line_thickness);
        // rankle-rknee
        if(person.right_ankle.confidence > 0 && person.right_knee.confidence > 0)
            cv::line(edited_img, right_ankle, right_knee, line_color, line_thickness);
        // lhip-neck
        if(person.left_hip.confidence > 0 && person.neck.confidence > 0)
            cv::line(edited_img, left_hip, neck, line_color, line_thickness);
        // lknee-lhip
        if(person.left_knee.confidence > 0 && person.left_hip.confidence > 0)
            cv::line(edited_img, left_knee, left_hip, line_color, line_thickness);
        // lankle-lknee
        if(person.left_ankle.confidence > 0 && person.left_knee.confidence > 0)
            cv::line(edited_img, left_ankle, left_knee, line_color, line_thickness);
        // reye-nose
        if(person.right_eye.confidence > 0 && person.nose.confidence > 0)
            cv::line(edited_img, right_eye, nose, line_color, line_thickness);
        // leye-nose
        if(person.left_eye.confidence > 0 && person.nose.confidence > 0)
            cv::line(edited_img, left_eye, nose, line_color, line_thickness);
        // rear-reye
        if(person.right_ear.confidence > 0 && person.right_eye.confidence > 0)
            cv::line(edited_img, right_ear, right_eye, line_color, line_thickness);
        // lear-leye
        if(person.left_ear.confidence > 0 && person.left_eye.confidence > 0)
            cv::line(edited_img, left_ear, left_eye, line_color, line_thickness);
    }

    // Create appropriately sized openCV image
    cv::Mat display_img;
    cv::Size window_size(IMAGE_WIDTH, (int)(IMAGE_WIDTH * (float)edited_img.rows/(float)edited_img.cols));
    cv::resize(edited_img, display_img, window_size);


    cv::imshow(WINDOW_NAME, display_img);
    cv::waitKey(1);
}

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
void camera_cb(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        //cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        return;
    }
    if (cv_ptr->image.empty()) return;

    cv::Mat img_mat = cv_ptr->image.clone();

    //==================================================
    //============== OPENPOSE PROCESSING ===============
    //==================================================

    const op::Point<int> imageSize{img_mat.cols, img_mat.rows};
    // Step 2 - Get desired scale sizes
    std::vector<double> scaleInputToNetInputs;
    std::vector<op::Point<int>> netInputSizes;
    double scaleInputToOutput;
    op::Point<int> outputResolution;
    std::tie(scaleInputToNetInputs, netInputSizes, scaleInputToOutput, outputResolution)
        = scaleAndSizeExtractor->extract(imageSize);
    // Step 3 - Format input image to OpenPose input and output formats
    netInputArray = cvMatToOpInput.createArray(img_mat, scaleInputToNetInputs, netInputSizes);
    outputArray = cvMatToOpOutput.createArray(img_mat, scaleInputToOutput, outputResolution);
    // Step 4 - Estimate poseKeypoints
    poseExtractorPtr->forwardPass(netInputArray, imageSize, scaleInputToNetInputs);
    const auto poseKeypoints = poseExtractorPtr->getPoseKeypoints();
    const auto scaleNetToOutput = poseExtractorPtr->getScaleNetToOutput();
    // Step 5 - Render pose
    poseGpuRenderer->renderPose(outputArray, poseKeypoints, scaleInputToOutput, scaleNetToOutput);
    // Step 6 - OpenPose output format to cv::Mat
    auto outputImage = opOutputToCvMat.formatToCvMat(outputArray);

    // publish skeleton info.
    std_msgs::Header header = msg->header;

    campose_msgs::FramePoses framePosesMsg;
    framePosesMsg.header = header;
    framePosesMsg.poses.resize(poseKeypoints.getSize(0));

    for (auto person = 0; person < poseKeypoints.getSize(0); person++) {
        fill_pose_with_keypoints(framePosesMsg.poses[person], poseKeypoints, person);
    }
    
    person_keypoint_pub.publish(framePosesMsg);

    display_person_image(img_mat, framePosesMsg);
}

// Heavily based on ildoonet's ros-openpose node here: https://github.com/ildoonet/ros-openpose/blob/master/openpose_ros_node/src/openpose_ros_node.cpp
int main(int argc, char* argv[]) {
    ROS_INFO("Starting program...");

    cv::namedWindow(WINDOW_NAME);

    gflags::ParseCommandLineFlags(&argc, &argv, true);

    // ROS initialization
    ros::init(argc, argv, "golem_campose_node");
    ros::NodeHandle nh;
    ros::Rate r(10);

    std::string camera_topic;
    nh.param<std::string>("camera_topic", camera_topic, "/flycap_cam/image");
    nh.param<int>("image_width", IMAGE_WIDTH, 900);


    // Supress OpenPose logging
    FLAGS_logtostderr = false;
    //google::InitGoogleLogging(argv[0]);
    op::ConfigureLog::setPriorityThreshold(op::Priority::NoOutput);
    op::Profiler::setDefaultX(1000);

    const auto outputSize = op::flagsToPoint(FLAGS_output_resolution, "-1x-1");
    // netInputSize
    const auto netInputSize = op::flagsToPoint(FLAGS_net_resolution, "-1x368");
    // poseModel
    const auto poseModel = op::flagsToPoseModel(FLAGS_model_pose);
    // Check no contradictory flags enabled
    if (FLAGS_alpha_pose < 0. || FLAGS_alpha_pose > 1.)
        op::error("Alpha value for blending must be in the range [0,1].", __LINE__, __FUNCTION__, __FILE__);
    if (FLAGS_scale_gap <= 0. && FLAGS_scale_number > 1)
        op::error("Incompatible flag configuration: scale_gap must be greater than 0 or scale_number = 1.",
                  __LINE__, __FUNCTION__, __FILE__);

    scaleAndSizeExtractor = new op::ScaleAndSizeExtractor(netInputSize, outputSize, FLAGS_scale_number, FLAGS_scale_gap);
    poseExtractorPtr = std::make_shared<op::PoseExtractorCaffe>(poseModel, FLAGS_model_folder,
                                                                     FLAGS_num_gpu_start);
    poseGpuRenderer = new op::PoseGpuRenderer(poseModel, poseExtractorPtr, (float)FLAGS_render_threshold,
                                              !FLAGS_disable_blending, (float)FLAGS_alpha_pose, (float)FLAGS_alpha_heatmap);
    poseGpuRenderer->setElementToRender(FLAGS_part_to_show);
    op::FrameDisplayer frameDisplayer("OpenPose Tutorial - Example 2", outputSize);
    // Step 4 - Initialize resources on desired thread (in this case single thread, i.e. we init resources here)
    poseExtractorPtr->initializationOnThread();
    poseGpuRenderer->initializationOnThread();


    ROS_INFO("Configuration:");
    ROS_INFO("Camera topic:  '%s'", camera_topic.c_str());
    ROS_INFO("Image width:    %d", IMAGE_WIDTH);
    ROS_INFO("Models folder: '%s'", FLAGS_model_folder.c_str());
    ROS_INFO("Pose model:    '%s'", FLAGS_model_pose.c_str());
    ROS_INFO("Net resolution: %dx%d", netInputSize.x, netInputSize.y);


    ros::Subscriber image_sub = nh.subscribe(camera_topic, 1, camera_cb);
    person_keypoint_pub = nh.advertise<campose_msgs::FramePoses>("person_keypoints", 1000);
    
    ROS_INFO("Starting node.");

    ros::spin();
}