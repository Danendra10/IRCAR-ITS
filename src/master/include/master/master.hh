#ifndef __MASTER_HH_
#define __MASTER_HH_

#include "ros/package.h"
#include "ros/ros.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/UInt8.h"

#include "msg_collection/CmdVision.h"
#include "msg_collection/Obstacles.h"
#include "msg_collection/RealPosition.h"
#include "msg_collection/SlopeIntercept.h"

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

typedef struct general_data_tag {
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
    ros::Publisher pub_cmd_vision;
    ros::Subscriber sub_car_pose;
    ros::Subscriber sub_lines;
    ros::Subscriber sub_real_lines;
    ros::Subscriber sub_road_sign;
    ros::Subscriber sub_car_data;
    ros::Subscriber sub_lidar_data;
    ros::Subscriber sub_stop_signal;
    ros::Timer tim_60_hz;

    std::vector<cv::Vec4i> vision_data_lane;

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

const string commands[] = { "stop", "right", "left", "forward", "no entry", "right", "start tunnel", "stop" };

//==============================================================================

extern PID_Const pid_linear_const;
extern PID_Const pid_angular_const;
extern bool linear_negative;
extern bool angular_negative;
//==============================================================================

void CllbckTim60Hz(const ros::TimerEvent& event);

void CllbckSubLidarData(const msg_collection::Obstacles::ConstPtr& msg)
{
    general_instance.raw_obs_data.clear();
    general_instance.obs_data.clear();

    if (msg->x.size() == 0) {
        general_instance.obs_status = false;
        return;
    }

    for (int i = 0; i < msg->x.size(); i++) {
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

void CllbckSubCarData(const sensor_msgs::JointState::ConstPtr& msg, general_data_ptr general_instance)
{
    // posisi sumbu x
    general_instance->car_data.rear_left_wheel_joint = msg->position[0];
    general_instance->car_data.rear_right_wheel_joint = msg->position[1];
    general_instance->car_data.front_right_wheel_joint = msg->position[4];
    general_instance->car_data.front_left_wheel_joint = msg->position[5];
    general_instance->car_data.vel_front_left = msg->velocity[2];
    general_instance->car_data.vel_front_right = msg->velocity[3];

    // general_instance->car_data.distance_between_wheels = fabs(general_instance->car_data.front_right_wheel_joint - general_instance->car_data.front_left_wheel_joint);
}

void CllbckSubCarPose(const geometry_msgs::Point::ConstPtr& msg)
{
    general_instance.car_pose.x = msg->x;
    general_instance.car_pose.y = msg->y;
    general_instance.car_pose.th = msg->z;
}

void CllbckSubRealLaneVector(const msg_collection::RealPosition::ConstPtr& msg)
{

    general_instance.buffer_target_x = msg->target_x;
    general_instance.buffer_target_y = msg->target_y;
    general_instance.can_be_intercepted = msg->can_be_intercepted;

    general_instance.left_available = false;
    general_instance.middle_available = false;
    general_instance.right_available = false;

    if (msg->left_lane_x_top != 0
        && msg->left_lane_x_bottom != 0
        && msg->left_lane_y_top != 0
        && msg->left_lane_y_bottom != 0) {
        general_instance.left_available = true;
    }

    if (msg->middle_lane_x_top != 0
        && msg->middle_lane_x_bottom != 0
        && msg->middle_lane_y_top != 0
        && msg->middle_lane_y_bottom != 0) {
        general_instance.middle_available = true;
    }

    if (msg->right_lane_x_top != 0
        && msg->right_lane_x_bottom != 0
        && msg->right_lane_y_top != 0
        && msg->right_lane_y_bottom != 0) {
        general_instance.right_available = true;
    }

    // if (general_instance.left_available)
    //     Logger(YELLOW, "Left Available");
    // if (general_instance.middle_available)
    //     Logger(YELLOW, "Middle Available");
    // if (general_instance.right_available)
    //     Logger(YELLOW, "Right Available");
    float spacer_x = 800 - 600;
    float spacer_y = 30;
    float dist_x_left = 800 - 600;
    float dist_y_left = (msg->left_lane_x_bottom + msg->left_lane_x_top) / 2.0 - 400;
    float dist_x_middle = 800 - 600;
    float dist_y_middle = (msg->middle_lane_x_bottom + msg->middle_lane_x_top) / 2.0 - 400;
    float dist_x_right = 800 - 600;
    float dist_y_right = (msg->right_lane_x_bottom + msg->right_lane_x_top) / 2.0 - 400;

    float distance_spacer = pixel_to_real(sqrt(pow(spacer_x, 2) + pow(spacer_y, 2)));
    float distance_left = pixel_to_real(sqrt(pow(dist_x_left, 2) + pow(dist_y_left, 2)));
    float distance_middle = pixel_to_real(sqrt(pow(dist_x_middle, 2) + pow(dist_y_middle, 2)));
    float distance_right = pixel_to_real(sqrt(pow(dist_x_right, 2) + pow(dist_y_right, 2)));
    float angle_diff_spacer = atan(spacer_x / spacer_y);
    float angle_diff_left = atan(dist_x_left / dist_y_left);
    float angle_diff_middle = atan(dist_x_middle / dist_y_middle);
    float angle_diff_right = atan(dist_x_right / dist_y_right);

    if (spacer_y < 0)
        angle_diff_spacer += DEG2RAD(180);
    if (dist_y_left < 0)
        angle_diff_left += DEG2RAD(180);
    if (dist_y_right < 0)
        angle_diff_right += DEG2RAD(180);
    if (dist_y_middle < 0)
        angle_diff_middle += DEG2RAD(180);

    general_instance.spacer_real_x = distance_spacer * sin(angle_diff_spacer);
    general_instance.spacer_real_y = distance_spacer * cos(angle_diff_spacer);
    general_instance.car_target_left.x = distance_left * sin(angle_diff_left);
    general_instance.car_target_left.y = distance_left * cos(angle_diff_left);
    general_instance.car_target_middle.x = distance_middle * sin(angle_diff_middle);
    general_instance.car_target_middle.y = distance_middle * cos(angle_diff_middle);
    general_instance.car_target_right.x = distance_right * sin(angle_diff_right);
    general_instance.car_target_right.y = distance_right * cos(angle_diff_right);

    // general_instance.car_target_left.x = dist_x_left;
    // general_instance.car_target_left.y = dist_y_left;
    // general_instance.car_target_middle.x = dist_x_middle;
    // general_instance.car_target_middle.y = dist_y_middle;
    // general_instance.car_target_right.x = dist_x_right;
    // general_instance.car_target_right.y = dist_y_right;

    // Logger(MAGENTA, "left x : %f left y : %f", dist_x_left, dist_y_left);
    // Logger(MAGENTA, "middle x : %f middle y : %f", dist_x_middle, dist_y_middle);
    // Logger(MAGENTA, "right x : %f right y : %f", dist_x_right, dist_y_right);

    if (general_instance.left_available
        && general_instance.middle_available
        && general_instance.right_available) {
        general_instance.divider = (int)(abs(general_instance.car_target_left.y - general_instance.car_target_right.y) / 4.0);
    }

    data_validator |= 0b001;
}

void CllbckSubRoadSign(const std_msgs::UInt16ConstPtr& msg)
{
    general_instance.sign_type = msg->data;
    data_validator |= 0b100;
}

void CllbckSubSignalStop(const std_msgs::UInt8ConstPtr& msg, general_data_ptr general_instance)
{
    general_instance->signal_stop = msg->data;
    // printf("signal stop %d\n", general_instance->signal_stop);
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

    if (!initialized) {
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