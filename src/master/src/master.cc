#include "master/master.hh"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "master");
    ros::NodeHandle NH;
    ros::MultiThreadedSpinner MTS;

    general_instance.pub_car_vel = NH.advertise<geometry_msgs::Twist>("/catvehicle/cmd_vel_safe", 10);

    general_instance.sub_car_pose = NH.subscribe("car_pose", 1, CllbckSubCarPose);
    general_instance.sub_lines = NH.subscribe("lines", 1, CllbckSubLaneVector);
    general_instance.sub_car_data = NH.subscribe<sensor_msgs::JointState>("/catvehicle/joint_states", 1, boost::bind(CllbckSubCarData, _1, &general_instance));

    general_instance.tim_60_hz = NH.createTimer(ros::Duration(1.0 / 60.0), CllbckTim60Hz);

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

void DecideCarTarget(general_data_ptr data)
{
    if (data->middle_lane.size() < 0)
        return;
    if (data->middle_lane[0].x == 0)
        return;

    data->car_target.x = data->middle_lane[0].x;
    data->car_target.y = data->middle_lane[0].y;
    data->car_target.th = atan2(data->car_target.y - data->car_pose.y, data->car_target.x - data->car_pose.x);
    printf("car_target.th: %f\n", data->car_target.th);
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

    printf("alpha: %f\n", alpha);
}

void TransmitData(general_data_ptr data)
{
    geometry_msgs::Twist vel_msg;
    vel_msg.angular.z = data->car_vel.th;
    vel_msg.linear.x = data->car_vel.x;
    data->pub_car_vel.publish(vel_msg);
}