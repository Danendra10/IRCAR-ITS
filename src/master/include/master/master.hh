#ifndef __MASTER_HH_
#define __MASTER_HH_

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "entity/entity.hh"
#include "msg_collection/PointArray.h"
#include <vector>

#include <termios.h>
#include <sys/ioctl.h>

#include "master/MachineState.hh"

using namespace std;

typedef struct general_data_tag
{
    Velocity car_vel;
    ros::Publisher pub_car_vel;

    ros::Subscriber sub_car_pose;
    ros::Subscriber sub_lines;

    ros::Timer tim_60_hz;

    CarPose car_pose;
    vector<Lane> left_lane;
    vector<Lane> middle_lane;
    vector<Lane> right_lane;

    vector<Lane> middle_right_target;
    vector<Lane> middle_left_target;

    MachineState main_state;

} general_data_t, *general_data_ptr;

//==============================================================================

general_data_t general_instance;

//==============================================================================

void CllbckTim60Hz(const ros::TimerEvent &event);

void CllbckSubCarPose(const geometry_msgs::Point::ConstPtr &msg)
{
    general_instance.car_pose.x = msg->x;
    general_instance.car_pose.y = msg->y;
    general_instance.car_pose.th = msg->z;
}

void CllbckSubLaneVector(const msg_collection::PointArray::ConstPtr &msg)
{
    general_instance.left_lane.clear();
    general_instance.middle_lane.clear();
    general_instance.right_lane.clear();
    general_instance.middle_right_target.clear();
    general_instance.middle_left_target.clear();

    for (int i = 0; i < msg->left_lane_x.size(); i++)
    {
        Lane lane;
        lane.x = msg->left_lane_x[i];
        lane.y = msg->left_lane_y[i];
        general_instance.left_lane.push_back(lane);
    }

    for (int i = 0; i < msg->middle_lane_x.size(); i++)
    {
        Lane lane;
        lane.x = msg->middle_lane_x[i];
        lane.y = msg->middle_lane_y[i];
        general_instance.middle_lane.push_back(lane);
    }

    for (int i = 0; i < msg->right_lane_x.size(); i++)
    {
        Lane lane;
        lane.x = msg->right_lane_x[i];
        lane.y = msg->right_lane_y[i];
        general_instance.right_lane.push_back(lane);
    }

    for (int i = 0; i < msg->left_target_x.size(); i++)
    {
        Lane lane;
        lane.x = msg->left_target_x[i];
        lane.y = msg->left_target_y[i];
        general_instance.middle_left_target.push_back(lane);
    }

    for (int i = 0; i < msg->right_target_x.size(); i++)
    {
        Lane lane;
        lane.x = msg->right_target_x[i];
        lane.y = msg->right_target_y[i];
        general_instance.middle_right_target.push_back(lane);
    }
}

//==============================================================================

void GetKeyboard();
void SimulatorState();
void MoveRobot(float vx_, float vz_);
void RobotMovement(TargetPtr target, VelocityPtr vel);
void TransmitData(general_data_ptr data);

int8_t kbhit()
{
    static const int STDIN = 0;
    static bool initialized = false;

    if (!initialized)
    {
        termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initialized = true;
    }

    int bytesWaiting;
    ioctl(STDIN, FIONREAD, &bytesWaiting);
    return bytesWaiting;
}

#endif