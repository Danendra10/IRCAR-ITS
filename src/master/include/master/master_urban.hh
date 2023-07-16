#ifndef __MASTER_URBAN_HH_
#define __MASTER_URBAN_HH_

#include "ros/package.h"
#include "ros/ros.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"

#include "msg_collection/Obstacles.h"

#include "yaml-cpp/yaml.h"
#include <exception>
#include <iostream>
#include <string>
#include <sys/ioctl.h>
#include <termios.h>
#include <vector>

#include "entity/entity.hh"
#include "logger/logger.h"
#include "master/MachineState.hh"
#include "math/math.hh"
#include "motion/motion.hh"

using namespace std;

typedef struct general_data_tag
{
    Velocity car_vel;
    CarPose car_pose;
    Target car_target;
    Target car_target_left;
    Target car_target_middle;
    Target car_target_right;
    CarData car_data;

    vector<Obstacles> raw_obs_data;
    vector<Obstacles> obs_data;

    ros::Publisher pub_car_vel;

    ros::Subscriber sub_car_pose;
    ros::Subscriber sub_road_sign;
    ros::Subscriber sub_odom_data;
    ros::Subscriber sub_lidar_data;
    ros::Subscriber sub_stop_signal;
    ros::Subscriber sub_vision_road_sign_py;
    ros::Subscriber sub_vision_angle_error;

    ros::Timer tim_60_hz;

    MachineState main_state;

    bool obs_status;
    uint8_t car_side;
    uint8_t moved_state;
    uint16_t sign_type;
    uint16_t prev_sign_type;
    uint16_t lock_state;
    uint8_t signal_stop;

    bool can_be_intercepted;
    bool left_available;
    bool middle_available;
    bool right_available;

    float buffer_target_x;
    float buffer_target_y;

    float spacer_real_x;
    float spacer_real_y;

    float prev_x;
    float prev_y;

    bool last_lidar_status;

    int divider;

    String road_sign_from_model;

    float angle_error;

} general_data_t, *general_data_ptr;

//==============================================================================

general_data_t general_instance;

/**
 * 0b000 ==> !data valdi
 * 0b001 ==> data vision valdi
 * 0b011 ==> data lidar valid
 * 0b111 ==> data road sign valid
 */
uint8_t data_validator = 0b000;

//==============================================================================

const string commands[] = {"stop", "right", "left", "forward", "no entry", "right", "start tunnel", "stop"};

//==============================================================================

extern PID_Const pid_linear_const;
extern PID_Const pid_angular_const;
extern bool linear_negative;
extern bool angular_negative;
//==============================================================================

void CllbckTim60Hz(const ros::TimerEvent &event);

void CllbckSubLidarData(const msg_collection::Obstacles::ConstPtr &msg)
{
    general_instance.raw_obs_data.clear();
    general_instance.obs_data.clear();

    if (msg->x.size() == 0)
    {
        general_instance.obs_status = false;
        return;
    }

    for (int i = 0; i < msg->x.size(); i++)
    {
        Obstacles raw_obs;
        raw_obs.x = msg->x[i];
        raw_obs.y = msg->y[i];
        general_instance.raw_obs_data.push_back(raw_obs);
        // float dst = sqrt(pow(raw_obs.x, 2) + pow(raw_obs.y, 2));
        // if (i % 5 == 0)
        //     printf("lidar || x %.2f y %.2f dist %f\n", raw_obs.x, raw_obs.y, dst);

        Obstacles obs;
        obs.x = msg->x[i] + general_instance.car_pose.x;
        obs.y = msg->y[i] + general_instance.car_pose.y;
        general_instance.obs_data.push_back(obs);

        general_instance.obs_status = true;
    }

    // printf("obs status %d\n", general_instance.obs_status);

    data_validator |= 0b010;
}

void CllbckSubCarPose(const nav_msgs::Odometry::ConstPtr &msg)
{
    general_instance.car_pose.x = msg->pose.pose.position.x;
    general_instance.car_pose.y = msg->pose.pose.position.y;
    general_instance.car_pose.th = RAD2DEG(2 * atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
}

void CllbckSubRoadSign(const std_msgs::UInt16ConstPtr &msg)
{
    general_instance.sign_type = msg->data;
    data_validator |= 0b100;
}

void CllbckSubSignalStop(const std_msgs::UInt8ConstPtr &msg, general_data_ptr general_instance)
{
    general_instance->signal_stop = msg->data;
    // printf("signal stop %d\n", general_instance->signal_stop);
}

void CllbckSubVisionRoadSignPy(const std_msgs::StringConstPtr &msg, general_data_ptr general_instance)
{
    general_instance->road_sign_from_model = msg->data;
    printf("road sign from model %s\n", general_instance->road_sign_from_model.c_str());
}

void CllbckAngleError(const std_msgs::Float32ConstPtr &msg, general_data_ptr general_instance)
{
    general_instance->angle_error = msg->data;
}

//==============================================================================

void GetKeyboard();
void SimulatorState();
void MoveRobot(float vx_, float vz_);
void TransmitData(general_data_ptr data);
void RobotMovement(general_data_ptr data);
void DecideCarTarget(general_data_ptr data);
void AutoDrive(general_data_ptr data);
void TurnCarLeft90Degree(general_data_ptr general_data);
void TurnCarRight90Degree(general_data_ptr general_data);
void KeepForward(general_data_ptr general_data);
bool TurnCarRight90Degree2(general_data_ptr general_data, float steering, float time_to_turn);
void StopRobot(general_data_ptr data);
bool StopRobot(general_data_ptr data, float time_to_stop);
void UrbanMovement(general_data_ptr data);
int MasterInit();
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