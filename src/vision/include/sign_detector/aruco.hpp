#ifndef __ARUCO_HH_
#define __ARUCO_HH_
#include "ros/package.h"
#include "ros/ros.h"

#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"

#include "std_msgs/UInt16.h"
#include "std_msgs/UInt8.h"

#include "yaml-cpp/yaml.h"
#include <iostream>
#include <string>

using namespace std;
using namespace cv;

const string commands[] = {"stop", "right", "left", "forward", "no entry", "right", "start tunnel", "stop"};

//---Timer
ros::Timer tim_30hz;
//---Publisher
ros::Publisher pub_detected_sign_data;
ros::Publisher pub_signal_stop;
//---Subscriber
image_transport::Subscriber sub_raw_frame;

//---Matrix
cv::Mat frame_raw;
cv::Mat frame_gray;
cv::Mat thresholded;
cv::Mat output_image = frame_raw.clone();

//---Aruco
std::vector<int> marker_ids;
std::vector<std::vector<cv::Point2f>> marker_corners, rejected_candidates;
cv::aruco::DetectorParameters detector_params;
cv::Ptr<cv::aruco::DetectorParameters> detector_params_ptr;
cv::Ptr<cv::aruco::Dictionary> dictionary;

//====================================================================================================

boost::mutex mutex_raw_frame;

//====================================================================================================

int validator;
int counter = 0;
float distance_to_detected_sign;
uint8_t threshold_counter_road_sign;
int distance_to_road_sign_threshold;
int x_pos_road_sign_threshold;
int y_pos_road_sign_threshold;
uint8_t stop_signal;
int center_cam_x = 400;
int center_cam_y = 800;

void CallbackSubRawFrame(const sensor_msgs::ImageConstPtr &msg);
void CallbackTimer30Hz(const ros::TimerEvent &event);

void LoadConfig();
int ArucoInit();
#endif