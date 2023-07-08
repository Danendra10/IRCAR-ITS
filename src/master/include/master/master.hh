#ifndef __MASTER_HH_
#define __MASTER_HH_

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "entity/entity.hh"
#include "msg_collection/PointArray.h"
#include "msg_collection/Obstacles.h"
#include "msg_collection/RealPosition.h"
#include "sensor_msgs/JointState.h"
#include "math/math.hh"
#include "std_msgs/UInt16.h"

#include <vector>

#include <termios.h>
#include <sys/ioctl.h>

#include "master/MachineState.hh"
#include "motion/motion.hh"
#include "logger/logger.h"

using namespace std;

typedef struct general_data_tag
{
    Velocity car_vel;
    CarPose car_pose;
    Target car_target;
    CarData car_data;

    vector<Obstacles> raw_obs_data;
    vector<Obstacles> obs_data;

    ros::Publisher pub_car_vel;
    ros::Subscriber sub_car_pose;
    ros::Subscriber sub_lines;
    ros::Subscriber sub_real_lines;
    ros::Subscriber sub_road_sign;
    ros::Subscriber sub_car_data;
    ros::Subscriber sub_lidar_data;
    ros::Timer tim_60_hz;

    vector<Lane> left_lane;
    vector<Lane> middle_lane;
    vector<Lane> right_lane;

    vector<RealLane> left_lane_real;
    vector<RealLane> middle_lane_real;
    vector<RealLane> right_lane_real;

    MachineState main_state;

    uint8_t obs_status;
    uint8_t car_side;
    uint8_t moved_state;
    uint16_t sign_type;

} general_data_t, *general_data_ptr;

//==============================================================================

general_data_t general_instance;

/**
 * 0b000 ==> !data valdi
 * 0b001 ==> data vision valdi
 * 0b011 ==> data lidar valdi
 */
uint8_t data_validator = 0b000;

//==============================================================================

void CllbckTim60Hz(const ros::TimerEvent &event);

void CllbckSubLidarData(const msg_collection::Obstacles::ConstPtr &msg)
{
    general_instance.raw_obs_data.clear();
    general_instance.obs_data.clear();

    if (msg->x.size() == 0)
    {
        general_instance.obs_status = 0;
        return;
    }

    for (int i = 0; i < msg->x.size(); i++)
    {
        Obstacles raw_obs;
        raw_obs.x = msg->x[i];
        raw_obs.y = msg->y[i];
        general_instance.raw_obs_data.push_back(raw_obs);
        // printf("lidar || x %.2f y %.2f\n", raw_obs.x, raw_obs.y);

        Obstacles obs;
        obs.x = msg->x[i] + general_instance.car_pose.x;
        obs.y = msg->y[i] + general_instance.car_pose.y;
        general_instance.obs_data.push_back(obs);
        if (obs.y > 0)
            general_instance.obs_status = 1;
        else
            general_instance.obs_status = 2;
    }

    data_validator |= 0b010;
    // printf("data validnya %d\n", data_validator);
}

void CllbckSubCarData(const sensor_msgs::JointState::ConstPtr &msg, general_data_ptr general_instance)
{
    // posisi sumbu x
    general_instance->car_data.rear_left_wheel_joint = msg->position[0];
    general_instance->car_data.rear_right_wheel_joint = msg->position[1];
    general_instance->car_data.front_right_wheel_joint = msg->position[4];
    general_instance->car_data.front_left_wheel_joint = msg->position[5];

    // general_instance->car_data.distance_between_wheels = fabs(general_instance->car_data.front_right_wheel_joint - general_instance->car_data.front_left_wheel_joint);
}

void CllbckSubCarPose(const geometry_msgs::Point::ConstPtr &msg)
{
    general_instance.car_pose.x = msg->x;
    general_instance.car_pose.y = msg->y;
    general_instance.car_pose.th = msg->z;
}

void CllbckSubRealLaneVector(const msg_collection::RealPosition::ConstPtr &msg)
{
    general_instance.left_lane_real.clear();
    general_instance.middle_lane_real.clear();
    general_instance.right_lane_real.clear();

    for (int i = 0; i < msg->left_lane_x_real.size(); i++)
    {
        RealLane real_lane;
        real_lane.x = msg->left_lane_x_real[i];
        real_lane.y = msg->left_lane_y_real[i];
        general_instance.left_lane_real.push_back(real_lane);
    }

    for (int i = 0; i < msg->right_lane_x_real.size(); i++)
    {
        RealLane real_lane;
        real_lane.x = msg->right_lane_x_real[i];
        real_lane.y = msg->right_lane_y_real[i];
        general_instance.right_lane_real.push_back(real_lane);
    }

    for (int i = 0; i < msg->middle_lane_x_real.size(); i++)
    {
        RealLane real_lane;
        real_lane.x = msg->middle_lane_x_real[i];
        real_lane.y = msg->middle_lane_y_real[i];
        general_instance.middle_lane_real.push_back(real_lane);
    }
}

void CllbckSubLaneVector(const msg_collection::PointArray::ConstPtr &msg)
{
    int buffer_lane_x, buffer_lane_y;
    general_instance.left_lane.clear();
    general_instance.middle_lane.clear();
    general_instance.right_lane.clear();

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

    data_validator |= 0b001;
}

void CllbckSubRoadSign(const std_msgs::UInt16ConstPtr &msg)
{
    general_instance.sign_type = msg->data;
}

//==============================================================================

void GetKeyboard();
void SimulatorState();
void MoveRobot(float vx_, float vz_);
void TransmitData(general_data_ptr data);
void RobotMovement(general_data_ptr data);
void DecideCarTarget(general_data_ptr data);
void AutoDrive(general_data_ptr data);
void MoveWithObs(general_data_ptr data_in_obs);

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