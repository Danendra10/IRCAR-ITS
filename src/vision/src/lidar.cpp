/**
 * @copyright @isabellejt
 * @brief This node will calculate the line of the road
 *       and publish the point of the line
 * @license IRIS
 * TODO: #2 Find a way to convert the detected obstacle to the pixel of frame @isabellejt
 */

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

ros::Timer tim_50_hz;

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

    tim_50_hz = nh.createTimer(ros::Duration(1.0 / 50.0), CllbckMainTimer);

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
    // printf("car pose || %.2f %.2f %.2f\n", car_pose.x, car_pose.y, car_pose.th);
}

void SubLidarCllbck(const sensor_msgs::LaserScanConstPtr &msg)
{
    msg_collection::Obstacles raw_obstacles;
    ranges = msg->ranges;
    for (int i = 0; i < ranges.size(); i++)
    {
        float curr_range = ranges[i];
        float obs_x, obs_y;
        if (ranges[i] < 40)
        {
            // based on car perspective
            obs_y = cos(DEG2RAD(i)) * curr_range;
            obs_x = sin(DEG2RAD(i)) * curr_range;
            // printf("x: %f y %f\n", obs_x, obs_y);
            printf("x: %f y %f dist %f\n", obs_x, obs_y,curr_range);
            raw_obstacles.x.push_back(obs_x);
            raw_obstacles.y.push_back(obs_y);
            raw_obstacles.dist.push_back(curr_range);
        }
    }
    pub_lidar_data.publish(raw_obstacles);
}