
#include <iostream>
#include <string>
#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"
#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"

using namespace std;
using namespace cv;

const string commands[] = {"dead end", "end tunnel", "forward", "left", "no entry", "right", "start tunnel", "stop"};

ros::Timer tim_30hz;
ros::Subscriber sub_raw_frame;

cv::Mat frame_raw;
cv::Mat frame_gray;
cv::Mat output_image = frame_raw.clone();
std::vector<int> marker_ids;
std::vector<std::vector<cv::Point2f>> marker_corners, rejected_candidates;
cv::aruco::DetectorParameters detector_params = cv::aruco::DetectorParameters();
// cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_250);
cv::Ptr<cv::aruco::DetectorParameters> detector_params_ptr = cv::makePtr<cv::aruco::DetectorParameters>(detector_params);
cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);

void CallbackSubRawFrame(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        frame_raw = cv_bridge::toCvShare(msg, "bgr8")->image;
        // cv::cvtColor(frame_raw, frame_gray, cv::COLOR_BGR2GRAY);
        // cv::aruco::detectMarkers(frame_gray, dictionary, marker_corners, marker_ids, detector_params_ptr, rejected_candidates);
        // cv::aruco::drawDetectedMarkers(output_image, marker_corners, marker_ids);
        // cv::imshow("Raw Frame", output_image);
        // cv::waitKey(1);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void CallbackTimer30Hz(const ros::TimerEvent &event)
{
    Mat image = imread("/home/iris/routine/assets/generate_images/1/output_image.jpg");

    // cv::cvtColor(image, frame_gray, cv::COLOR_BGR2GRAY);

    cv::aruco::detectMarkers(image, dictionary, marker_corners, marker_ids, detector_params_ptr, rejected_candidates);

    cv::aruco::drawDetectedMarkers(image, marker_corners, marker_ids);

    printf("Detected %lu markers\n", marker_ids.size());

    for (int i = 0; i < marker_ids.size(); ++i)
    {
        int id = marker_ids[i];
        cv::Point2f center = (marker_corners[i][0] + marker_corners[i][1] + marker_corners[i][2] + marker_corners[i][3]) / 4;
        cv::putText(image, commands[id - 1], center, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
    }

    imshow("Raw Frame", image);
    waitKey(1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "road_sign_detector");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner spinner(4);
    image_transport::ImageTransport it(nh);

    // ros::Subscriber sub_raw_frame = nh.subscribe("/camera/image_raw", 1, SubRawFrameCllbck);
    tim_30hz = nh.createTimer(ros::Duration(1.0 / 30.0), CallbackTimer30Hz);

    spinner.spin();
    return 0;
}