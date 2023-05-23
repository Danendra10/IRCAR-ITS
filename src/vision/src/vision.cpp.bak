/**
 * @copyright @Danendra10 & @isabellejt
 * @brief This node will calculate the line of the road
 *       and publish the point of the line
 * @license IRIS
 * TODO: #1 make the wrap based on the pdf that been approved by the Mr. Pandu @Danendra10
 */

#include "opencv2/opencv.hpp"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include "image_transport/image_transport.h"
#include <chrono>
#include <vector>
#include "nav_msgs/Odometry.h"
#include "entity/entity.hh"
#include "geometry_msgs/Point.h"
#include "msg_collection/PointArray.h"
#include "msg_collection/Obstacles.h"

#define RAD2DEG(rad) ((rad)*180.0 / M_PI)
#define DEG2RAD(deg) ((deg)*M_PI / 180.0)

//============================================================

using namespace std;
using namespace cv;

//============================================================

image_transport::Subscriber sub_raw_frame;

ros::Subscriber sub_odom;
ros::Subscriber sub_lidar_data;

ros::Publisher pub_car_pose;
ros::Publisher pub_points;

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

//============================================================

void SubRawFrameCllbck(const sensor_msgs::ImageConstPtr &msg);
void SubOdomRaw(const nav_msgs::Odometry::ConstPtr &msg);
void SubLidarData(const msg_collection::Obstacles::ConstPtr &msg);

//============================================================

void Tim30HzCllbck(const ros::TimerEvent &event);

//============================================================

void Init();
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision");
    ros::NodeHandle NH;
    image_transport::ImageTransport IT(NH);
    ros::MultiThreadedSpinner MTS(0);

    ROS_INFO("HALO\n");

    tim_30hz = NH.createTimer(ros::Duration(1.0 / 30.0), Tim30HzCllbck);

    sub_raw_frame = IT.subscribe("/catvehicle/camera_front/image_raw_front", 1, SubRawFrameCllbck);
    sub_odom = NH.subscribe("/catvehicle/odom", 1, SubOdomRaw);
    sub_lidar_data = NH.subscribe("/lidar_data", 1, SubLidarData);

    pub_car_pose = NH.advertise<geometry_msgs::Point>("/car_pose", 1);
    pub_points = NH.advertise<msg_collection::PointArray>("/lines", 1);

    MTS.spin();

    return 0;
}

void SubRawFrameCllbck(const sensor_msgs::ImageConstPtr &msg)
{
    mutex_raw_frame.lock();
    raw_frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    mutex_raw_frame.unlock();

    validators |= 0b001;
}

void SubOdomRaw(const nav_msgs::Odometry::ConstPtr &msg)
{
    car_pose.x = msg->pose.pose.position.x;
    car_pose.y = msg->pose.pose.position.y;
    car_pose.th = RAD2DEG(2 * atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));

    geometry_msgs::Point car_pose_msg;
    car_pose_msg.x = car_pose.x;
    car_pose_msg.y = car_pose.y;
    car_pose_msg.z = car_pose.th;
    pub_car_pose.publish(car_pose_msg);
}

void SubLidarData(const msg_collection::Obstacles::ConstPtr &msg)
{
    obstacles.clear();
    raw_obstacles.clear();
    for (int i = 0; i < msg->x.size(); i++)
    {
        ObstaclesPtr obstacle(new Obstacles);
        obstacle->x = msg->x[i];
        obstacle->y = msg->y[i];
        obstacles.push_back(obstacle);

        ObstaclesPtr raw_obstacle(new Obstacles);
        raw_obstacle->x = msg->x[i] - car_pose.x;
        raw_obstacle->y = msg->y[i] - car_pose.y;
        raw_obstacles.push_back(raw_obstacle);
    }
}

void Tim30HzCllbck(const ros::TimerEvent &event)
{
    if (validators != 0b001)
        return;

    Mat wrapped_frame;
    Mat obs_frame;
    vector<Vec4i> lines;
    vector<Point> points;

    static vector<Point> prev_left_points;
    static vector<Point> prev_right_points;
    static vector<Point> prev_middle_points;

    vector<Point> middle_left;
    vector<Point> middle_right;

    vector<Vec4i> left_lines;
    vector<Vec4i> right_lines;
    vector<Vec4i> middle_lines;

    vector<Point> left_extrapolation_points;
    vector<Point> right_extrapolation_points;

    wrapped_frame = ToWrappedFrame(raw_frame);
    points = GetPoints(wrapped_frame);
    left_points = GetLeftPoints(points);
    right_points = GetRightPoints(points, wrapped_frame.cols);
    middle_points = GetMiddlePoints(points, wrapped_frame.cols);
    middle_left = GetMiddleOfLeftRoad(left_points, middle_points);
    middle_right = GetMiddleOfRightRoad(right_points, middle_points);
    obs_frame = DrawObsPoints(obstacles);

    for (int i = 0; i < left_points.size(); i++)
    {
        Point p = left_points[i];
        circle(wrapped_frame, p, 5, Scalar(0, 0, 255), -1);
    }

    for (int i = 0; i < right_points.size(); i++)
    {
        Point p = right_points[i];
        circle(wrapped_frame, p, 5, Scalar(0, 255, 0), -1);
    }

    for (int i = 0; i < middle_points.size(); i++)
    {
        Point p = middle_points[i];
        circle(wrapped_frame, p, 5, Scalar(255, 0, 0), -1);
    }

    for (int i = 0; i < middle_left.size(); i++)
    {
        Point p = middle_left[i];
        circle(wrapped_frame, p, 5, Scalar(255, 255, 0), -1);
    }

    for (int i = 0; i < middle_right.size(); i++)
    {
        Point p = middle_right[i];
        circle(wrapped_frame, p, 5, Scalar(255, 0, 255), -1);
    }

    msg_collection::PointArray points_msg;
    for (int i = 0; i < left_points.size(); i++)
    {
        Point p = left_points[i];
        points_msg.left_lane_x.push_back(p.x);
        points_msg.left_lane_y.push_back(p.y);
    }

    for (int i = 0; i < right_points.size(); i++)
    {
        Point p = right_points[i];
        points_msg.right_lane_x.push_back(p.x);
        points_msg.right_lane_y.push_back(p.y);
    }

    for (int i = 0; i < middle_points.size(); i++)
    {
        Point p = middle_points[i];
        points_msg.middle_lane_x.push_back(p.x);
        points_msg.middle_lane_y.push_back(p.y);
    }

    for (int i = 0; i < middle_left.size(); i++)
    {
        Point p = middle_left[i];
        points_msg.left_target_x.push_back(p.x);
        points_msg.left_target_y.push_back(p.y);
    }

    for (int i = 0; i < middle_right.size(); i++)
    {
        Point p = middle_right[i];
        points_msg.right_target_x.push_back(p.x);
        points_msg.right_target_y.push_back(p.y);
    }

    pub_points.publish(points_msg);

    imshow("frame", raw_frame);
    imshow("wrapped_frame", wrapped_frame);
    imshow("Obs_frame", obs_frame);
    waitKey(1);
}

//========================================================================================================================

void Init()
{
    curr_time = std::chrono::system_clock::now();
}

Mat ToWrappedFrame(Mat raw_frame)
{
    Point pt_a = Point(2, 450);
    Point pt_b = Point(2, 670);
    Point pt_c = Point(798, 670);
    Point pt_d = Point(798, 450);

    float width_AD = sqrt(pow(pt_a.x - pt_d.x, 2) + pow(pt_a.y - pt_d.y, 2));
    float width_BC = sqrt(pow(pt_b.x - pt_c.x, 2) + pow(pt_b.y - pt_c.y, 2));

    float height_AB = sqrt(pow(pt_a.x - pt_b.x, 2) + pow(pt_a.y - pt_b.y, 2));
    float height_CD = sqrt(pow(pt_c.x - pt_d.x, 2) + pow(pt_c.y - pt_d.y, 2));

    vector<Point2f> input_pts;
    input_pts.push_back(pt_a);
    input_pts.push_back(pt_b);
    input_pts.push_back(pt_c);
    input_pts.push_back(pt_d);

    vector<Point2f> output_pts;
    output_pts.push_back(Point2f(0, 0));
    output_pts.push_back(Point2f(0, 800));
    output_pts.push_back(Point2f(800, 800));
    output_pts.push_back(Point2f(800, 0));

    Mat mask = getPerspectiveTransform(input_pts, output_pts);

    Mat wrapped_frame;

    warpPerspective(raw_frame, wrapped_frame, mask, Size(800, 800));

    return wrapped_frame;
}

vector<Point> GetPoints(Mat wrapped_frame)
{
    Mat gray, blur, edges, mask, masked_edges;

    vector<Point> points;

    cvtColor(wrapped_frame, gray, COLOR_BGR2GRAY);

    GaussianBlur(gray, blur, Size(5, 5), 0);

    Canny(blur, edges, 50, 150);

    float height = wrapped_frame.rows;
    float width = wrapped_frame.cols;

    vector<Point> roi_vertices;
    roi_vertices.push_back(Point(0, height));
    roi_vertices.push_back(Point(0, height * 1 / 5));
    roi_vertices.push_back(Point(width, height * 1 / 5));
    roi_vertices.push_back(Point(width, height));

    mask = Mat::zeros(edges.size(), edges.type());

    fillConvexPoly(mask, roi_vertices, Scalar(255, 255, 255));

    for (int i = 0; i < edges.rows; i++)
    {
        for (int j = 0; j < edges.cols; j++)
        {
            if (edges.at<uchar>(i, j) == 255)
            {
                points.push_back(Point(j, i));
            }
        }
    }

    return points;
}

std::vector<cv::Vec4i> GetLeftLines(const std::vector<cv::Vec4i> &lines)
{
    std::vector<cv::Vec4i> left_lines;
    int min_x = 800; // Initialize with a large value

    for (const cv::Vec4i &line : lines)
    {
        cv::Point p1(line[0], line[1]);
        cv::Point p2(line[2], line[3]);

        // Check if the line is within the left region
        if (p1.x <= min_x && p2.x <= min_x && p1.x <= 300 && p2.x <= 300)
        {
            // Check if the line is close to the previous line
            if (left_lines.empty() || std::abs(p1.x - min_x) <= 50)
            {
                left_lines.push_back(line);
                min_x = std::min(p1.x, p2.x);
            }
        }
    }

    return left_lines;
}

std::vector<cv::Vec4i> GetRightLines(const std::vector<cv::Vec4i> &lines, int frameWidth)
{
    std::vector<cv::Vec4i> right_lines;
    int max_x = 0; // Initialize with a small value

    for (const cv::Vec4i &line : lines)
    {
        cv::Point p1(line[0], line[1]);
        cv::Point p2(line[2], line[3]);

        // Check if the line is within the right region
        if (p1.x >= max_x && p2.x >= max_x && p1.x >= frameWidth - 300 && p2.x >= frameWidth - 300)
        {
            // Check if the line is close to the previous line
            if (right_lines.empty() || std::abs(p1.x - max_x) <= 50)
            {
                right_lines.push_back(line);
                max_x = std::max(p1.x, p2.x);
            }
        }
    }

    return right_lines;
}

std::vector<cv::Vec4i> GetMiddleLines(const std::vector<cv::Vec4i> &lines, int frameWidth)
{
    std::vector<cv::Vec4i> middle_lines;
    int min_x = frameWidth / 2 - 100; // Left boundary of middle region
    int max_x = frameWidth / 2 + 100; // Right boundary of middle region

    for (const cv::Vec4i &line : lines)
    {
        cv::Point p1(line[0], line[1]);
        cv::Point p2(line[2], line[3]);

        // Check if the line is within the middle region
        if ((p1.x >= min_x && p1.x <= max_x) || (p2.x >= min_x && p2.x <= max_x))
        {
            if (middle_lines.empty() || std::abs(p1.x - p2.x) <= 50)
            {
                middle_lines.push_back(line);
            }
        }
    }

    return middle_lines;
}

std::vector<cv::Vec4i> GetMiddlePoints(const std::vector<cv::Vec4i> &leftLines, const std::vector<cv::Vec4i> &middleLines)
{
    std::vector<cv::Vec4i> middlePoints;

    for (size_t i = 0; i < leftLines.size() && i < middleLines.size(); ++i)
    {
        cv::Point2f leftLineStart(leftLines[i][0], leftLines[i][1]);
        cv::Point2f leftLineEnd(leftLines[i][2], leftLines[i][3]);

        cv::Point2f middleLineStart(middleLines[i][0], middleLines[i][1]);
        cv::Point2f middleLineEnd(middleLines[i][2], middleLines[i][3]);

        cv::Point2f middlePt = 0.5f * (leftLineStart + middleLineStart);

        middlePoints.push_back(cv::Vec4i(middlePt.x, middlePt.y));
    }

    return middlePoints;
}

cv::Vec4i ExtrapolateLine(const cv::Vec4i &line, int minY, int maxY)
{
    double slope = static_cast<double>(line[3] - line[1]) / static_cast<double>(line[2] - line[0]);
    int startX = line[0] + static_cast<int>((minY - line[1]) / slope);
    int endX = line[0] + static_cast<int>((maxY - line[1]) / slope);
    return cv::Vec4i(startX, minY, endX, maxY);
}

std::vector<cv::Point> GetLeftPoints(const std::vector<cv::Point> &points)
{
    std::vector<cv::Point> left_points;
    int min_x = 800; // Initialize with a large value

    for (const cv::Point &point : points)
    {
        if (point.y < 100)
            continue;
        // Check if the point is within the left region
        if (point.x <= min_x && point.x <= 300)
        {
            // Check if the point is close to the previous point
            if (left_points.empty() || std::abs(point.x - min_x) <= 50)
            {
                left_points.push_back(point);
                min_x = point.x;
            }
        }
    }

    return left_points;
}

std::vector<cv::Point> GetRightPoints(const std::vector<cv::Point> &points, int frameWidth)
{
    std::vector<cv::Point> right_points;
    int max_x = 0; // Initialize with a small value

    for (const cv::Point &point : points)
    {
        if (point.y < 100)
            continue;
        // Check if the point is within the right region
        if (point.x >= max_x && point.x >= frameWidth - 300)
        {
            // Check if the point is close to the previous point
            if (right_points.empty() || std::abs(point.x - max_x) <= 50)
            {
                right_points.push_back(point);
                max_x = point.x;
            }
        }
    }

    return right_points;
}

std::vector<cv::Point> GetMiddlePoints(const std::vector<cv::Point> &points, int frameWidth)
{
    std::vector<cv::Point> middle_points;
    int min_x = frameWidth / 2 - 100; // Left boundary of middle region
    int max_x = frameWidth / 2 + 100; // Right boundary of middle region

    for (const cv::Point &point : points)
    {
        if (point.y < 100)
            continue;
        // check if the point exist in the left and right region
        for (const cv::Point &left_point : left_points)
        {
            if (point.x == left_point.x && point.y == left_point.y)
            {
                continue;
            }
        }
        for (const cv::Point &right_point : right_points)
        {
            if (point.x == right_point.x && point.y == right_point.y)
            {
                continue;
            }
        }
        if (point.x >= min_x && point.x <= max_x)
        {

            if (middle_points.empty() || std::abs(point.x - point.x) <= 50)
            {
                middle_points.push_back(point);
            }
        }
    }

    return middle_points;
}

std::vector<cv::Point> GetMiddleOfLeftRoad(const std::vector<cv::Point> &leftPoints, const std::vector<cv::Point> &middlePoints)
{
    std::vector<cv::Point> middleOfLeftRoad;

    for (size_t i = 0; i < leftPoints.size() && i < middlePoints.size(); ++i)
    {
        cv::Point2f leftPoint(leftPoints[i].x, leftPoints[i].y);
        cv::Point2f middlePoint(middlePoints[i].x, middlePoints[i].y);

        cv::Point2f middlePt = 0.5f * (leftPoint + middlePoint);

        middleOfLeftRoad.push_back(middlePt);
    }

    return middleOfLeftRoad;
}

std::vector<cv::Point> GetMiddleOfRightRoad(const std::vector<cv::Point> &rightPoints, const std::vector<cv::Point> &middlePoints)
{
    std::vector<cv::Point> middleOfRightRoad;

    for (size_t i = 0; i < rightPoints.size() && i < middlePoints.size(); ++i)
    {
        cv::Point2f rightPoint(rightPoints[i].x, rightPoints[i].y);
        cv::Point2f middlePoint(middlePoints[i].x, middlePoints[i].y);

        cv::Point2f middlePt = 0.5f * (rightPoint + middlePoint);

        middleOfRightRoad.push_back(middlePt);
    }

    return middleOfRightRoad;
}

Mat DrawObsPoints(const vector<ObstaclesPtr> &points)
{
    Mat frame = Mat::zeros(800, 800, CV_8UC3);
    // draw car in the middle
    float car_x = 400;
    float car_y = 700;

    cv::rectangle(frame, cv::Point(car_x - 20, car_y - 20), cv::Point(car_x + 20, car_y + 20), cv::Scalar(0, 255, 0), 2);

    for (int i = 0; i < points.size(); i++)
    {
        // float obs_x = (points[i]->y * 10) + 400;
        // float obs_y = (points[i]->x * -10) + 700;
        float obs_y = (points[i]->x * 8 + car_x);
        float obs_x = (points[i]->y * 8 + car_y);

        printf("obs frame x %.2f y %.2f\n", obs_x, obs_y);
        cv::circle(frame, cv::Point(obs_x, obs_y), 5, cv::Scalar(0, 0, 255), -1);
    }

    for (int i = 0; i < middle_points.size(); i++)
    {
        cv::circle(frame, cv::Point(middle_points[i].x, middle_points[i].y), 5, cv::Scalar(255, 0, 0), -1);
    }

    // robot safe areas
    // float safe_angle = 10;
    // cv::line(frame, cv::Point(300, 400), cv::Point(400, 700), cv::Scalar(255, 0, 0), 2);
    // cv::line(frame, cv::Point(500, 400), cv::Point(400, 700), cv::Scalar(255, 0, 0), 2);

    return frame;
}