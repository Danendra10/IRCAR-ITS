
#include <iostream>
#include <string>
#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"
#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "std_msgs/UInt16"

using namespace std;
using namespace cv;

const string commands[] = {"dead end", "end tunnel", "forward", "left", "no entry", "right", "start tunnel", "stop"};

ros::Timer tim_30hz;
ros::Subscriber sub_raw_frame;
ros::Publisher pub_detected_sign_data;

cv::Mat frame_raw;
cv::Mat frame_gray;
cv::Mat output_image = frame_raw.clone();
std::vector<int> marker_ids;
std::vector<std::vector<cv::Point2f>> marker_corners, rejected_candidates;
cv::aruco::DetectorParameters detector_params = cv::aruco::DetectorParameters();
// cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_250);
cv::Ptr<cv::aruco::DetectorParameters> detector_params_ptr = cv::makePtr<cv::aruco::DetectorParameters>(detector_params);
cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);

void CallbackSubRawFrame(const sensor_msgs::ImageConstPtr &msg);
void CallbackTimer30Hz(const ros::TimerEvent &event);