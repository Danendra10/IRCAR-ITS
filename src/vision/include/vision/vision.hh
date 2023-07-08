#ifndef __VISION_HH_
#define __VISION_HH_

#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include "image_transport/image_transport.h"
#include <chrono>
#include "nav_msgs/Odometry.h"
#include "entity/entity.hh"
#include "geometry_msgs/Point.h"
#include "msg_collection/PointArray.h"
#include "msg_collection/Obstacles.h"
#include "msg_collection/RealPosition.h"
#include "imp/imph.hh"
#include "math/math.hh"
#include "vision/LaneDetect.hh"
#include "logger/logger.h"
#include "pinhole/pinhole.hh"

#define RAD2DEG(rad) ((rad)*180.0 / M_PI)
#define DEG2RAD(deg) ((deg)*M_PI / 180.0)
#define DST_REMAPPED_WIDTH 800
#define DST_REMAPPED_HEIGHT 800
#define SRC_RESIZED_WIDTH 800
#define SRC_RESIZED_HEIGHT 800

#define DEGREE 2

//============================================================

using namespace std;
using namespace cv;

//============================================================

image_transport::Subscriber sub_raw_frame;

ros::Subscriber sub_odom;
ros::Subscriber sub_lidar_data;

ros::Publisher pub_car_pose;
ros::Publisher pub_points;
ros::Publisher pub_lane;

ros::Timer tim_30hz;

//============================================================

Mat raw_frame = Mat::zeros(800, 800, CV_8UC3);

//============================================================

boost::mutex mutex_raw_frame;

//============================================================

uint8_t validators = 0b000;
std::chrono::time_point<std::chrono::system_clock> curr_time;
CarPose car_pose;

vector<ObstaclesPtr> raw_obstacles;
vector<ObstaclesPtr> obstacles;

vector<Point> left_points;
vector<Point> right_points;
vector<Point> middle_points;

Mat wrapped_frame;
Mat resized;
Mat grayresized;
// Mat imremapped = Mat(DST_REMAPPED_HEIGHT, DST_REMAPPED_WIDTH, CV_8UC1);

PolynomialRegression polynom(DEGREE);

//============================================================

void SubRawFrameCllbck(const sensor_msgs::ImageConstPtr &msg);
void SubOdomRaw(const nav_msgs::Odometry::ConstPtr &msg);
void SubLidarData(const msg_collection::Obstacles::ConstPtr &msg);

//============================================================

void Tim30HzCllbck(const ros::TimerEvent &event);
void click_event(int event, int x, int y, int flags, void *params);

//============================================================

void Init();
void record();
Mat ToWrappedFrame(Mat raw_frame);
vector<Point> GetPoints(Mat wrapped_frame);
std::vector<cv::Point> GetLeftPoints(const std::vector<cv::Point> &points);
std::vector<cv::Point> GetRightPoints(const std::vector<cv::Point> &points, int frameWidth);
std::vector<cv::Point> GetMiddlePoints(const std::vector<cv::Point> &points, int frameWidth);
std::vector<cv::Point> GetMiddleOfLeftRoad(const std::vector<cv::Point> &leftPoints, const std::vector<cv::Point> &middlePoints);
std::vector<cv::Point> GetMiddleOfRightRoad(const std::vector<cv::Point> &rightPoints, const std::vector<cv::Point> &middlePoints);

Mat DrawObsPoints(const vector<ObstaclesPtr> &points);

std::vector<cv::Vec4i> GetLeftLines(const std::vector<cv::Vec4i> &lines);
std::vector<cv::Vec4i> GetRightLines(const std::vector<cv::Vec4i> &lines, int frameWidth);
std::vector<cv::Vec4i> GetMiddleLines(const std::vector<cv::Vec4i> &lines, int frameWidth);
std::vector<cv::Vec4i> GetMiddlePoints(const std::vector<cv::Vec4i> &leftLines, const std::vector<cv::Vec4i> &middleLines);
cv::Vec4i ExtrapolateLine(const cv::Vec4i &line, int minY, int maxY);

//============================================================

#endif