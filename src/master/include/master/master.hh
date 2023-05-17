#ifndef __MASTER_HH_
#define __MASTER_HH_

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "entity/entity.hh"
#include "msg_collection/PointArray.h"
#include <vector>

using namespace std;

ros::Subscriber sub_car_pose;
ros::Subscriber sub_lines;

ros::Timer tim_60_hz;

//==============================================================================

CarPose car_pose;
vector<Lane> left_lane;
vector<Lane> middle_lane;
vector<Lane> right_lane;

vector<Lane> middle_right_target;
vector<Lane> middle_left_target;

//==============================================================================

void CllbckTim60Hz(const ros::TimerEvent &event);

void CllbckSubCarPose(const geometry_msgs::Point::ConstPtr &msg)
{
    car_pose.x = msg->x;
    car_pose.y = msg->y;
    car_pose.th = msg->z;
}

void CllbckSubLaneVector(const msg_collection::PointArray::ConstPtr &msg)
{
    left_lane.clear();
    middle_lane.clear();
    right_lane.clear();
    middle_right_target.clear();
    middle_left_target.clear();

    for (int i = 0; i < msg->left_lane_x.size(); i++)
    {
        Lane lane;
        lane.x = msg->left_lane_x[i];
        lane.y = msg->left_lane_y[i];
        printf("pose %f %f\n", lane.x, lane.y);
        left_lane.push_back(lane);
    }

    for (int i = 0; i < msg->middle_lane_x.size(); i++)
    {
        Lane lane;
        lane.x = msg->middle_lane_x[i];
        lane.y = msg->middle_lane_y[i];
        middle_lane.push_back(lane);
    }

    for (int i = 0; i < msg->right_lane_x.size(); i++)
    {
        Lane lane;
        lane.x = msg->right_lane_x[i];
        lane.y = msg->right_lane_y[i];
        right_lane.push_back(lane);
    }

    for (int i = 0; i < msg->left_target_x.size(); i++)
    {
        Lane lane;
        lane.x = msg->left_target_x[i];
        lane.y = msg->left_target_y[i];
        middle_left_target.push_back(lane);
    }

    for (int i = 0; i < msg->right_target_x.size(); i++)
    {
        Lane lane;
        lane.x = msg->right_target_x[i];
        lane.y = msg->right_target_y[i];
        middle_right_target.push_back(lane);
    }
}

#endif