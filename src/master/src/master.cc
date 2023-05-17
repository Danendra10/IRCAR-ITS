#include "master/master.hh"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "master");
    ros::NodeHandle NH;
    ros::MultiThreadedSpinner MTS;

    general_instance.pub_car_vel = NH.advertise<geometry_msgs::Twist>("/catvehicle/cmd_vel_safe", 10);

    general_instance.sub_car_pose = NH.subscribe("car_pose", 1, CllbckSubCarPose);
    general_instance.sub_lines = NH.subscribe("lines", 1, CllbckSubLaneVector);

    general_instance.tim_60_hz = NH.createTimer(ros::Duration(1.0 / 60.0), CllbckTim60Hz);

    MTS.spin();
    return 0;
}

void CllbckTim60Hz(const ros::TimerEvent &event)
{
    GetKeyboard();
    SimulatorState();
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

        case ' ':
            general_instance.main_state.value = RESET;
            break;
        }
    }
}

void MoveRobot(float vx_, float vz_)
{
    geometry_msgs::Twist vel_msg;
    vel_msg.angular.z = vz_;
    vel_msg.linear.x = vx_;
    general_instance.pub_car_vel.publish(vel_msg);
}

void SimulatorState()
{
    switch (general_instance.main_state.value)
    {
    case FORWARD:
        MoveRobot(2, 0);
        break;

    case BACKWARD:
        MoveRobot(-2, 0);
        break;

    case TURN_LEFT:
        MoveRobot(2, 2);
        break;

    case TURN_RIGHT:
        MoveRobot(2, -2);
        break;

    case RESET:
        MoveRobot(0, 0);
        break;

    default:
        MoveRobot(0, 0);
        break;
    }
}

void RobotMovement(TargetPtr target, VelocityPtr vel)
{
}

void TransmitData(general_data_ptr data)
{
    geometry_msgs::Twist vel_msg;
    vel_msg.angular.z = data->car_vel.th;
    vel_msg.linear.x = data->car_vel.x;
    data->pub_car_vel.publish(vel_msg);
}