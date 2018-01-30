#include <ros/ros.h>

#include <openpose/core/headers.hpp>
#include <openpose/filestream/headers.hpp>
#include <openpose/gui/headers.hpp>
#include <openpose/pose/headers.hpp>
#include <openpose/utilities/headers.hpp>

#include <opencv2/opencv.hpp>

#include <gflags/gflags.h>

#include <mutex>
#include <thread>

// Allow Google Flags in Ubuntu 14
#ifndef GFLAGS_GFLAGS_H_
    namespace gflags = google;
#endif

DEFINE_int32(logging_level,             3,              "The logging level. Integer in the range [0, 255]. 0 will output any log() message, while"
                                                        " 255 will not output any. Current OpenPose library messages are in the range 0-4: 1 for"
                                                        " low priority messages and 4 for important ones.");
// Producer
DEFINE_string(image_path,               "examples/media/COCO_val2014_000000000192.jpg",     "Process the desired image.");
// OpenPose
DEFINE_string(model_pose,               "MPI_4_layers", "Model to be used. E.g. `COCO` (18 keypoints), `MPI` (15 keypoints, ~10% faster), "
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

std::mutex outputImageMutex;
bool outputImageSet = false;
cv::Mat outputImage;

std::mutex latestImageMutex;
bool latestImageSet = false;
cv::Mat latestImage;

op::Point<int> outputSize;
op::Point<int> netInputSize;
op::PoseModel poseModel;

void process_image() {
    op::ScaleAndSizeExtractor scaleAndSizeExtractor(netInputSize, outputSize, FLAGS_scale_number, FLAGS_scale_gap);
    op::CvMatToOpInput cvMatToOpInput;
    op::CvMatToOpOutput cvMatToOpOutput;
    auto poseExtractorPtr = std::make_shared<op::PoseExtractorCaffe>(poseModel, FLAGS_model_folder,
                                                                     FLAGS_num_gpu_start);
    op::PoseGpuRenderer poseGpuRenderer{poseModel, poseExtractorPtr, (float)FLAGS_render_threshold,
                                        !FLAGS_disable_blending, (float)FLAGS_alpha_pose, (float)FLAGS_alpha_heatmap};
    poseGpuRenderer.setElementToRender(FLAGS_part_to_show);
    op::OpOutputToCvMat opOutputToCvMat;

    poseExtractorPtr->initializationOnThread();
    poseGpuRenderer.initializationOnThread();

    cv::Mat inputImage;

    while(ros::ok()) {
        bool skip = false;
        latestImageMutex.lock();
        skip = !latestImageSet;
        if(!skip)
            inputImage = latestImage.clone();
        latestImageMutex.unlock();

        if(skip) {
            usleep(100 * 1000); // Sleep for 100 ms before continuing
            continue;
        }

        if(inputImage.empty())
            op::error("Could not load frame from camera.", __LINE__, __FUNCTION__, __FILE__);
        
        const op::Point<int> imageSize{inputImage.cols, inputImage.rows};
        // Step 2 - Get desired scale sizes
        std::vector<double> scaleInputToNetInputs;
        std::vector<op::Point<int>> netInputSizes;
        double scaleInputToOutput;
        op::Point<int> outputResolution;
        std::tie(scaleInputToNetInputs, netInputSizes, scaleInputToOutput, outputResolution)
            = scaleAndSizeExtractor.extract(imageSize);
        // Step 3 - Format input image to OpenPose input and output formats
        const auto netInputArray = cvMatToOpInput.createArray(inputImage, scaleInputToNetInputs, netInputSizes);
        auto outputArray = cvMatToOpOutput.createArray(inputImage, scaleInputToOutput, outputResolution);
        // Step 4 - Estimate poseKeypoints
        poseExtractorPtr->forwardPass(netInputArray, imageSize, scaleInputToNetInputs);
        const auto poseKeypoints = poseExtractorPtr->getPoseKeypoints();
        const auto scaleNetToOutput = poseExtractorPtr->getScaleNetToOutput();
        // Step 5 - Render pose
        poseGpuRenderer.renderPose(outputArray, poseKeypoints, scaleInputToOutput, scaleNetToOutput);
        // Step 6 - OpenPose output format to cv::Mat
        outputImageMutex.lock();
        outputImage = opOutputToCvMat.formatToCvMat(outputArray);
        outputImageSet = true;
        outputImageMutex.unlock();
    }
}

int main(int argc, char* argv[]) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    // outputSize
    outputSize = op::flagsToPoint(FLAGS_output_resolution, "-1x-1");
    // netInputSize
    netInputSize = op::flagsToPoint(FLAGS_net_resolution, "-1x368");
    // poseModel
    poseModel = op::flagsToPoseModel(FLAGS_model_pose);
    // Check no contradictory flags enabled
    if (FLAGS_alpha_pose < 0. || FLAGS_alpha_pose > 1.)
        op::error("Alpha value for blending must be in the range [0,1].", __LINE__, __FUNCTION__, __FILE__);
    if (FLAGS_scale_gap <= 0. && FLAGS_scale_number > 1)
        op::error("Incompatible flag configuration: scale_gap must be greater than 0 or scale_number = 1.",
                  __LINE__, __FUNCTION__, __FILE__);

    op::FrameDisplayer frameDisplayer{"OpenPose Tutorial - Example 2", outputSize};

    ros::init(argc, argv, "golem_campose_node");

    ros::NodeHandle nh;
    ros::Rate r(30);

    std::thread process_thread(process_image);

    cv::VideoCapture cap;

    if(!cap.open(0)) {
        ROS_INFO_STREAM("There was an error loading the videocapture...");
        return -1;
    }

    while(ros::ok()) {
        ROS_INFO_STREAM("LOOP");
        latestImageMutex.lock();
        // TODO: Find a better solution. Thanks to http://answers.opencv.org/question/29957/highguivideocapture-buffer-introducing-lag/?answer=31513#post-id-31513
        for(int i=0; i<7; i++) { cap >> latestImage; }   // Temporary fix to clearing cap's buffer
        latestImageSet = true;
        latestImageMutex.unlock();

        cv::Mat output_image_copy;
        bool display = false;

        outputImageMutex.lock();
        display = outputImageSet;
        if(display)
            output_image_copy = outputImage.clone();
        outputImageMutex.unlock();

        if(display)
            frameDisplayer.displayFrame(output_image_copy, 1);

        r.sleep();
        ros::spinOnce();
    }
}