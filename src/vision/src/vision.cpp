#include "opencv2/opencv.hpp"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include "image_transport/image_transport.h"
#include <chrono>
#include <vector>

//============================================================

using namespace std;
using namespace cv;

//============================================================

image_transport::Subscriber sub_raw_frame;

ros::Timer tim_30hz;

//============================================================

Mat raw_frame = Mat::zeros(800, 800, CV_8UC3);

//============================================================

boost::mutex mutex_raw_frame;

//============================================================

uint8_t validators = 0b000;
// curr_time
std::chrono::time_point<std::chrono::system_clock> curr_time;

//============================================================

void SubRawFrameCllbck(const sensor_msgs::ImageConstPtr &msg);

//============================================================

void Tim30HzCllbck(const ros::TimerEvent &event);

//============================================================

void Init();
Mat ToWrappedFrame(Mat raw_frame);
vector<Vec4i> GetLines(Mat wrapped_frame);
vector<Vec4i> GetLeftLines(vector<Vec4i> lines);
vector<Vec4i> GetRightLines(vector<Vec4i> lines);
vector<Vec4i> GetMiddleLines(vector<Vec4i> lines);

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

void Tim30HzCllbck(const ros::TimerEvent &event)
{
    if (validators != 0b001)
        return;

    Mat wrapped_frame;
    vector<Vec4i> lines;

    wrapped_frame = ToWrappedFrame(raw_frame);
    lines = GetLines(wrapped_frame);

    for (int i = 0; i < lines.size(); i++)
    {
        Vec4i l = lines[i];
        line(wrapped_frame, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3, LINE_AA);
    }

    imshow("frame", raw_frame);
    imshow("wrapped_frame", wrapped_frame);
    waitKey(1);

    printf("validators: %d\n", validators);
}

//============================================================

void Init()
{
    curr_time = std::chrono::system_clock::now();
}

Mat ToWrappedFrame(Mat raw_frame)
{
    Point pt_a = Point(2, 414);
    Point pt_b = Point(2, 670);
    Point pt_c = Point(798, 670);
    Point pt_d = Point(798, 414);

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

vector<Vec4i> GetLines(Mat wrapped_frame)
{
    Mat gray, blur, edges, mask, masked_edges;

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

    bitwise_and(edges, mask, masked_edges);

    vector<Vec4i> lines;

    HoughLinesP(masked_edges, lines, 1, CV_PI / 180, 50, 50, 100);

    return lines;
}

vector<Vec4i> GetLeftLines(vector<Vec4i> lines)
{
    vector<Vec4i> left_lines;

    for (int i = 0; i < lines.size(); i++)
    {
        Vec4i l = lines[i];

        float slope = (float)(l[3] - l[1]) / (float)(l[2] - l[0]);

        if (slope < 0)
            left_lines.push_back(l);
    }

    return left_lines;
}

vector<Vec4i> GetRightLines(vector<Vec4i> lines)
{
    vector<Vec4i> right_lines;

    for (int i = 0; i < lines.size(); i++)
    {
        Vec4i l = lines[i];

        float slope = (float)(l[3] - l[1]) / (float)(l[2] - l[0]);

        if (slope > 0)
            right_lines.push_back(l);
    }

    return right_lines;
}

vector<Vec4i> GetMiddleLines(vector<Vec4i> lines)
{
    vector<Vec4i> middle_lines;

    for (int i = 0; i < lines.size(); i++)
    {
        Vec4i l = lines[i];

        float slope = (float)(l[3] - l[1]) / (float)(l[2] - l[0]);

        if (slope > -0.1 && slope < 0.1)
            middle_lines.push_back(l);
    }

    return middle_lines;
}