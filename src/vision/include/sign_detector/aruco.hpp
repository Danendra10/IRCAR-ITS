#ifndef __ARUCO_HH_
#define __ARUCO_HH_
#include <iostream>
#include <string>
#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"
#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "std_msgs/UInt16.h"
#include "yaml-cpp/yaml.h"
#include "ros/package.h"

using namespace std;
using namespace cv;

int validator;
uint8_t threshold_counter_road_sign;
int counter = 0;

const string commands[] = {"stop", "right", "left", "forward", "no entry", "right", "start tunnel", "stop"};

ros::Timer tim_30hz;
image_transport::Subscriber sub_raw_frame;
ros::Publisher pub_detected_sign_data;

cv::Mat frame_raw;
cv::Mat frame_gray;
cv::Mat output_image = frame_raw.clone();
std::vector<int> marker_ids;
std::vector<std::vector<cv::Point2f>> marker_corners, rejected_candidates;
cv::aruco::DetectorParameters detector_params;
// cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_250);
cv::Ptr<cv::aruco::DetectorParameters> detector_params_ptr;
cv::Ptr<cv::aruco::Dictionary> dictionary;

void CallbackSubRawFrame(const sensor_msgs::ImageConstPtr &msg);
void CallbackTimer30Hz(const ros::TimerEvent &event);
void LoadConfig();

int ArucoInit();
#endif