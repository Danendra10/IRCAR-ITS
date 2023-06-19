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
    general_instance.sub_real_lines = NH.subscribe("/real_lines", 1, CllbckSubRealLaneVector);

    general_instance.sub_car_data = NH.subscribe<sensor_msgs::JointState>("/catvehicle/joint_states", 1, boost::bind(CllbckSubCarData, _1, &general_instance));

    general_instance.tim_60_hz = NH.createTimer(ros::Duration(1 / 60), CllbckTim60Hz);

    MTS.spin();
    return 0;
}

void CllbckTim60Hz(const ros::TimerEvent &event)
{
    GetKeyboard();
    SimulatorState();
    // AutoDrive(&general_instance);
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
        MoveRobot(2, 0);
        break;

    case BACKWARD:
        MoveRobot(-2, 0);
        break;

    case TURN_LEFT:
        MoveRobot(1, 2);
        break;

    case TURN_RIGHT:
        MoveRobot(1, -2);
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
        int lane_buffer_x, lane_buffer_y;
        int middle_lane_size = general_data->middle_lane.size();
        int left_lane_size = general_data->left_lane.size();
        int right_lane_size = general_data->right_lane.size();
        float car_to_left = sqrt(pow(pixel_to_real(700 - general_data->left_lane[left_lane_size - 1].y), 2) + pow(pixel_to_real(400 - general_data->left_lane[left_lane_size - 1].x), 2));
        float car_to_right = sqrt(pow(pixel_to_real(700 - general_data->right_lane[right_lane_size - 1].y), 2) + pow(pixel_to_real(general_data->right_lane[right_lane_size - 1].x - 400), 2));
        int dist_between_points = general_data->middle_lane[middle_lane_size - 1].x - general_data->middle_lane[middle_lane_size - 5].x;

        // printf("from left %f || from right %f\n", car_to_left, car_to_right);

        if (middle_lane_size > 400 && abs(dist_between_points) < 20)
        {
            lane_buffer_x = general_data->middle_lane[middle_lane_size - 1].x;
            lane_buffer_y = general_data->middle_lane[middle_lane_size - 1].y;
        }
        else
        {
            general_data->middle_lane[middle_lane_size - 1].x = lane_buffer_x;
            general_data->middle_lane[middle_lane_size - 1].y = lane_buffer_y;
        }

        // ROS_ERROR("DIFF %d", dist_between_points);
        // MoveRobot(1, dist_between_points); //only for setting direction

        if ((car_to_left - car_to_right < -3 && general_data->obs_status == 0) || general_data->obs_status == 1)
            general_data->car_side = 10;
        else if ((car_to_left - car_to_right > 3 && general_data->obs_status == 0) || general_data->obs_status == 2)
            general_data->car_side = 20;

        switch (general_data->car_side)
        {
        case 10:
            // printf("TARGET KIRI\n");
            general_data->car_target.x = (general_data->left_lane_real[left_lane_size - 1].x + general_data->middle_lane_real[middle_lane_size - 1].x) / 2;
            general_data->car_target.y = (general_data->left_lane_real[left_lane_size - 1].y + general_data->middle_lane_real[middle_lane_size - 1].y) / 2;
            break;

        case 20:
            // printf("TARGET KANAN\n");
            general_data->car_target.x = (general_data->right_lane_real[right_lane_size - 1].x + general_data->middle_lane_real[middle_lane_size - 1].x) / 2;
            general_data->car_target.y = (general_data->right_lane_real[right_lane_size - 1].y + general_data->middle_lane_real[middle_lane_size - 1].y) / 2;
            break;

        default:
            // printf("TENGAH\n");
            general_data->car_target.x = general_data->middle_lane_real[middle_lane_size - 1].x;
            general_data->car_target.y = general_data->middle_lane_real[middle_lane_size - 1].y;
            break;
        }
        general_data->car_target.th = atan2(general_data->car_target.y - general_data->car_pose.y, general_data->car_target.x - general_data->car_pose.x);

        ROS_INFO("target %f %f %f\n", general_data->car_target.x, general_data->car_target.y, general_data->car_target.th);

        if (general_data->middle_lane.size() < 0)
            return;
        if (general_data->middle_lane[middle_lane_size - 1].x == 0)
            return;
    }
    catch (...)
    {
        printf("error\n");
    }
}

void RobotMovement(general_data_ptr data)
{
    // PURE PURSUIT
    // float y_from_center = 0.765 / 2;
    float rear_joint_y = (data->car_data.rear_left_wheel_joint + data->car_data.rear_right_wheel_joint) / 2;
    float rear_joint_x = rear_joint_y * tan(data->car_pose.th);
    printf("rear x %f y %f", rear_joint_x, rear_joint_y);
    float dist_y = data->car_target.y - rear_joint_y;
    float dist_x = data->car_target.x - rear_joint_x;

    float distance_between_wheels = (data->car_data.front_left_wheel_joint - data->car_data.rear_left_wheel_joint) / cos(data->car_pose.th);

    float ld = sqrt(pow(dist_x, 2) + pow(dist_y, 2));
    float alpha = atan(dist_y / dist_x);

    float delta = atan(2 * distance_between_wheels * sin(alpha) / ld);
    // printf("dist wheels %f delta %f \n", distance_between_wheels, delta);
    data->car_vel.th = delta;
    data->car_vel.x = 1;
}

void TransmitData(general_data_ptr data)
{
    geometry_msgs::Twist vel_msg;
    vel_msg.angular.z = data->car_vel.th;
    vel_msg.linear.x = data->car_vel.x;
    data->pub_car_vel.publish(vel_msg);
}