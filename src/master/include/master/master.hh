#ifndef __MASTER_HH_
#define __MASTER_HH_

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "entity/entity.hh"

using namespace std;

ros::Subscriber sub_car_pose;

//==============================================================================

CarPose car_pose;

//==============================================================================

void CllbckSubCarPose(const geometry_msgs::Point::ConstPtr &msg)
{
    car_pose.x = msg->x;
    car_pose.y = msg->y;
    car_pose.th = msg->z;
}

#endif