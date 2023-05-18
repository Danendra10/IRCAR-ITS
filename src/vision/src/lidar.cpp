#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <std_msgs/String.h>
#include "entity/entity.hh"
#include "msg_collection/Obstacles.h"

#define RAD2DEG(rad) ((rad)*180.0 / M_PI)
#define DEG2RAD(deg) ((deg)*M_PI / 180.0)

//============================================================

using namespace std;

//============================================================

std::vector<float> ranges;
CarPose car_pose;

//============================================================

ros::Timer tim_15_hz;

ros::Subscriber sub_odom;
ros::Subscriber sub_lidar;

ros::Publisher pub_lidar_data;

//============================================================

void SubOdomCllbck(const nav_msgs::OdometryConstPtr &odom);
void SubLidarCllbck(const sensor_msgs::LaserScanConstPtr &msg);

void CllbckMainTimer(const ros::TimerEvent &event);

//============================================================

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner MTS;

    tim_15_hz = nh.createTimer(ros::Duration(1.0 / 15.0), CllbckMainTimer);

    sub_odom = nh.subscribe("/catvehicle/odom", 1, SubOdomCllbck);
    sub_lidar = nh.subscribe("/catvehicle/front_laser_points", 1, SubLidarCllbck);

    pub_lidar_data = nh.advertise<msg_collection::Obstacles>("/lidar_data", 1);

    MTS.spin();
}

void CllbckMainTimer(const ros::TimerEvent &event)
{
    
}

void SubOdomCllbck(const nav_msgs::OdometryConstPtr &odom)
{
    car_pose.x = odom->pose.pose.position.x;
    car_pose.y = odom->pose.pose.position.y;
    car_pose.th = RAD2DEG(2 * atan2(odom->pose.pose.orientation.z, odom->pose.pose.orientation.w));
    if (car_pose.th < 0)
    {
        car_pose.th += 360;
    }
}

void SubLidarCllbck(const sensor_msgs::LaserScanConstPtr &msg)
{
    msg_collection::Obstacles obstacles;
    ranges = msg->ranges;
    for (int i = 0; i < ranges.size(); i++)
    {
        float curr_range = ranges[i];
        if (ranges[i] < 80)
        {
            obstacles.x.push_back(car_pose.y + sin(DEG2RAD(i)) * curr_range);
            obstacles.y.push_back(car_pose.x + cos(DEG2RAD(i)) * curr_range);
        }
    }
    pub_lidar_data.publish(obstacles);
}