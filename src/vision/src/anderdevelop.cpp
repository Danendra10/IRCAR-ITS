#include "imp/anderdevelop.hpp"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"

boost::mutex mutex_raw_frame;
uint8_t validators = 0b000;

void SubRawFrameCllbck(const sensor_msgs::ImageConstPtr &msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision");
    ros::NodeHandle NH;
    image_transport::ImageTransport IT(NH);
    ros::MultiThreadedSpinner MTS(0);

    image_transport::Subscriber sub_raw_frame = IT.subscribe("/catvehicle/camera_front/image_raw_front", 1, SubRawFrameCllbck);

    MTS.spin();

    return 0;
}

void SubRawFrameCllbck(const sensor_msgs::ImageConstPtr &msg)
{
    mutex_raw_frame.lock();
    raw_frame = cv_bridge::toCvShare(msg, "bgr8")->image;

    Mat transformation_matrix = K * (T * (R * A1));

    Mat bird_eye_view(raw_frame.size(), raw_frame.type());
    warpPerspective(raw_frame, bird_eye_view, transformation_matrix, Size(800, 800), INTER_CUBIC | WARP_INVERSE_MAP);
    mutex_raw_frame.unlock();

    validators |= 0b001;

    imshow("Bird's eye view", bird_eye_view);
    imshow("Raw frame", raw_frame);
    waitKey(1);
}
