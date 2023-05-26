#include "master/master.hh"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "master");
    ros::NodeHandle NH;
    ros::MultiThreadedSpinner MTS;

    general_instance.pub_car_vel = NH.advertise<geometry_msgs::Twist>("/catvehicle/cmd_vel_safe", 10);

    general_instance.sub_lidar_data = NH.subscribe("/lidar_data", 1, CllbckSubLidarData);
    general_instance.sub_car_pose = NH.subscribe("/car_pose", 1, CllbckSubCarPose);
    general_instance.sub_lines = NH.subscribe("/lines", 1, CllbckSubLaneVector);
    general_instance.sub_car_data = NH.subscribe<sensor_msgs::JointState>("/catvehicle/joint_states", 1, boost::bind(CllbckSubCarData, _1, &general_instance));

    general_instance.tim_60_hz = NH.createTimer(ros::Duration(1.0 / 50.0), CllbckTim60Hz);

    MTS.spin();
    return 0;
}

void CllbckTim60Hz(const ros::TimerEvent &event)
{
    GetKeyboard();
    SimulatorState();
    DecideCarTarget(&general_instance);
    TransmitData(&general_instance);
}

void GetKeyboard()
{
    static uint8_t prev_key = 0;
    if (kbhit() > 0)
    {
        char key = std::cin.get();

        switch (key)
        {
        case 'w':
            general_instance.main_state.value = FORWARD;
            break;
        case 's':
            general_instance.main_state.value = BACKWARD;
            break;
        case 'a':
            general_instance.main_state.value = TURN_LEFT;
            break;
        case 'd':
            general_instance.main_state.value = TURN_RIGHT;
            break;

        case 'o':
            general_instance.main_state.value = AUTONOMOUS;
            break;

        case ' ':
            general_instance.main_state.value = RESET;
            break;
        }
    }
}

void MoveRobot(float vx_, float vz_)
{
    general_instance.car_vel.x = vx_;
    general_instance.car_vel.th = vz_;
}

void SimulatorState()
{
    switch (general_instance.main_state.value)
    {
    case FORWARD:
        MoveRobot(1, 0);
        break;

    case BACKWARD:
        MoveRobot(-1, 0);
        break;

    case TURN_LEFT:
        MoveRobot(1, 1);
        break;

    case TURN_RIGHT:
        MoveRobot(1, -1);
        break;

    case AUTONOMOUS:
        RobotMovement(&general_instance);
        break;

    case RESET:
        MoveRobot(0, 0);
        break;

    default:
        MoveRobot(0, 0);
        break;
    }
}

void DecideCarTarget(general_data_ptr general_data)
{
    try
    {
        if (data_validator < 0b001)
            return;
        float car_to_left = sqrt(pow(general_data->car_pose.x - general_data->left_lane[0].x, 2) + pow(general_data->car_pose.y - general_data->left_lane[0].y, 2));
        float car_to_rght = sqrt(pow(general_data->car_pose.x - general_data->right_lane[0].x, 2) + pow(general_data->car_pose.y - general_data->right_lane[0].y, 2));

        if (general_data->obs_status)
        {
            float dist_from_left = sqrt(pow(general_data->left_lane[0].x - general_data->obs_data[0].x, 2) + pow(general_data->left_lane[0].y - general_data->obs_data[0].y, 2));
            float dist_from_rght = sqrt(pow(general_data->right_lane[0].x - general_data->obs_data[1].x, 2) + pow(general_data->right_lane[0].y - general_data->obs_data[1].y, 2));
            if (dist_from_left < dist_from_rght)
                general_data->car_side = 10; // in left
            else
                general_data->car_side = 20;

            switch (general_data->car_side)
            {
            case 10:
                printf("left\n");
                general_data->car_target.x = general_data->right_lane[0].x;
                general_data->car_target.y = general_data->right_lane[0].y;
                break;

                //====================================================================

            case 20:
                printf("right\n");
                general_data->car_target.x = general_data->left_lane[0].x;
                general_data->car_target.y = general_data->left_lane[0].y;
                break;
            }
            return;
        }

        if (general_data->middle_lane.size() < 0)
            return;
        if (general_data->middle_lane[0].x == 0)
            return;

        general_data->car_target.x = general_data->middle_lane[0].x;
        general_data->car_target.y = general_data->middle_lane[0].y;
        general_data->car_target.th = atan2(general_data->car_target.y - general_data->car_pose.y, general_data->car_target.x - general_data->car_pose.x);
    }
    catch (...)
    {
        printf("error\n");
    }
}

void RobotMovement(general_data_ptr data)
{
    //---Pure pirsuit ICR
    //===================
    float angle_to_target = atan2(data->car_target.y - data->car_pose.y, data->car_target.x - data->car_pose.x);
    float ld = 20;

    float alpha = (2 * data->car_data.distance_between_wheels * sin(angle_to_target - data->car_pose.th)) / ld;

    data->car_vel.th = alpha;
    data->car_vel.x = 0.5;

    // printf("target %.2f alpha: %f\n", data->car_target.y, alpha);
}

void TransmitData(general_data_ptr data)
{
    geometry_msgs::Twist vel_msg;
    vel_msg.angular.z = data->car_vel.th;
    vel_msg.linear.x = data->car_vel.x;
    data->pub_car_vel.publish(vel_msg);
}