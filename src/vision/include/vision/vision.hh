#ifndef _VISION_HH
#define _VISION_HH

#include "entity/entity.hh"
#include "geometry_msgs/Point.h"
#include "image_transport/image_transport.h"
#include "imp/imph.hh"
#include "logger/logger.h"
#include "math/math.hh"
#include "msg_collection/CmdVision.h"
#include "msg_collection/Obstacles.h"
#include "msg_collection/RealPosition.h"
#include "msg_collection/SlopeIntercept.h"
#include "nav_msgs/Odometry.h"
#include "pinhole/pinhole.hh"
#include "sensor_msgs/Image.h"
#include "std_msgs/Float32.h"
#include "vision/LaneDetect.hh"
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>

#define RAD2DEG(rad) ((rad)*180.0 / M_PI)
#define DEG2RAD(deg) ((deg)*M_PI / 180.0)
#define DST_REMAPPED_WIDTH 800
#define DST_REMAPPED_HEIGHT 800
#define SRC_RESIZED_WIDTH 800
#define SRC_RESIZED_HEIGHT 800

#define DEGREE 2

//==Method
// #define edge_detection

//============================================================

using namespace std;
using namespace cv;

//============================================================

image_transport::Subscriber sub_raw_frame;

ros::Subscriber sub_odom;
ros::Subscriber sub_cmd_vision;
ros::Subscriber sub_lidar_data;

ros::Publisher pub_car_pose;
ros::Publisher pub_target;
ros::Publisher pub_slope;

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

int target_x;
int target_y;
int x_target_left, x_target_right;
int y_target_left, y_target_right;
#define LeftLane true
#define RightLane false
bool decision = LeftLane;
#define Normal 0
#define LeftLost 1
#define RightLost 2
int8_t prev_state;
bool isWait = false;
int road_target = 1;
int prev_x_target = 6969;
int prev_spike = 3;
float x_target, y_target;
bool problem;
bool must3Lines = true;
bool find3Lines;
bool canbeIntercept = true;
double line_SI[3][2];
float prev_left[2];
float prev_middle[2];
float prev_right[2];

PolynomialRegression polynom(DEGREE);

//============================================================

void SubRawFrameCllbck(const sensor_msgs::ImageConstPtr &msg);
void SubOdomRaw(const nav_msgs::Odometry::ConstPtr &msg);
void SubLidarData(const msg_collection::Obstacles::ConstPtr &msg);
void SubCmdVision(const msg_collection::CmdVision::ConstPtr &msg);

//============================================================

void Tim30HzCllbck(const ros::TimerEvent &event);
void click_event(int event, int x, int y, int flags, void *params);

//============================================================

void Init();
void record();

void Detect(cv::Mat frame);
void ROI(cv::Mat &frame);
void Hough(cv::Mat frame, std::vector<cv::Vec4i> &line);
void Display(cv::Mat &frame, std::vector<cv::Vec4i> lines, int b_, int g_, int r_, float intensity);
void Average(cv::Mat frame, std::vector<cv::Vec4i> &lines);
void SlopeIntercept(cv::Vec4i &lines, double &slope, double &intercept);
cv::Vec2f VectorAvg(std::vector<cv::Vec2f> in_vec);
std::vector<cv::Vec4i> MakePoints(cv::Mat frame, cv::Vec2f lineSI);
std::vector<cv::Vec4i> SlidingWindows(cv::Mat &frame, std::vector<int> x_final, std::vector<int> nonzero_x, std::vector<int> nonzero_y);
void BinaryStacking(cv::Mat frame, cv::Mat &frame_dst);
void CenterSpike(cv::Mat frame, int start, int stop, int &index);

//============================================================

#endif