#ifndef URBAN_HPP
#define URBAN_HPP

#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include "opencv2/opencv.hpp"
#include "std_msgs/Float32.h"

using namespace cv;
using namespace std;

//----------------------------------------------

#define RAD2DEG(rad) ((rad)*180.0 / M_PI)
#define DEG2RAD(deg) ((deg)*M_PI / 180.0)

//----------------------------------------------

int thresh_min = 150;
int thresh_max = 255;

int h_min_lane = 0;
int h_max_lane = 180;
int s_min_lane = 0;
int s_max_lane = 18;
int v_min_lane = 111;
int v_max_lane = 255;

int h_min_road = 0;
int h_max_road = 180;
int s_min_road = 0;
int s_max_road = 255;
int v_min_road = 0;
int v_max_road = 146;

//----------------------------------------------

Mat raw_frame = Mat::zeros(800, 800, CV_8UC3);
Mat frame_thresholded_lane;
Mat frame_thresholded_road;

//----------------------------------------------

boost::mutex mutex_raw_frame;

//----------------------------------------------

uint8_t validators = 0b000;

image_transport::Subscriber sub_raw_frame;

ros::Publisher pub_error_angle;

ros::Timer tim_30hz;

void CllbckSubFrame(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        mutex_raw_frame.lock();
        raw_frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        mutex_raw_frame.unlock();
        validators |= 0b001;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void CllbckTim30Hz(const ros::TimerEvent &);

#endif