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
    general_instance.sub_road_sign = NH.subscribe("/vision/sign_detector/detected_sign_data", 1, CllbckSubRoadSign);

    general_instance.sub_car_data = NH.subscribe<sensor_msgs::JointState>("/catvehicle/joint_states", 1, boost::bind(CllbckSubCarData, _1, &general_instance));

    general_instance.tim_60_hz = NH.createTimer(ros::Duration(1 / 60), CllbckTim60Hz);

    MTS.spin();
    return 0;
}

void CllbckTim60Hz(const ros::TimerEvent &event)
{
    GetKeyboard();
    SimulatorState();
    AutoDrive(&general_instance);
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

void AutoDrive(general_data_ptr data)
{
    try
    {
        if (data_validator < 0b001)
        {
            Logger(RED, "Data hasn't been validated yet");
            return;
        }
        if (data->sign_type == NO_SIGN)
        {
            data->main_state.value = AUTONOMOUS_NO_SIGN;
        }
        else
        {
            if (data->sign_type == SIGN_STOP)
                data->main_state.value = AUTONOMOUS_STOP_SIGN;
            if (data->sign_type == SIGN_LEFT)
                data->main_state.value = AUTONOMOUS_TURN_LEFT_90;
            if (data->sign_type == SIGN_RIGHT)
                data->main_state.value = AUTONOMOUS_TURN_RIGHT_90;
            if (data->sign_type == SIGN_FORWARD)
                data->main_state.value = AUTONOMOUS_KEEP_FORWARD;
            if (data->sign_type == SIGN_DEAD_END)
                data->main_state.value = AUTONOMOUS_DEAD_END;
            if (data->sign_type == SIGN_NO_ENTRY)
                data->main_state.value = AUTONOMOUS_NO_ENTRY;
            if (data->sign_type == SIGN_START_TUNNEL)
                data->main_state.value = AUTONOMOUS_START_TUNNEL;
            if (data->sign_type == SIGN_END_TUNNEL)
                data->main_state.value = AUTONOMOUS_END_TUNNEL;
        }

        Logger(BLUE, "AutoDrive: %d", data->main_state.value);

        switch (data->main_state.value)
        {
        case AUTONOMOUS_NO_SIGN:
            RobotMovement(data);
            break;
        case AUTONOMOUS_STOP_SIGN:
            StopRobot(data);
            break;
        case AUTONOMOUS_TURN_LEFT_90:
            TurnCarLeft90Degree(data);
            break;
        case AUTONOMOUS_TURN_RIGHT_90:
            TurnCarRight90Degree(data);
            break;
        case AUTONOMOUS_KEEP_FORWARD:
            KeepForward(data);
            break;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
}

void StopRobot(general_data_ptr data)
{
    data->car_vel.x = 0;
    data->car_vel.th = 0;
}

/**
 * TODO: Check the sign distance, if it's too far, don't turn @danendra10
 */
void TurnCarLeft90Degree(general_data_ptr general_data)
{
    // Stop the car
    general_data->car_vel.x = 0;
    general_data->car_vel.th = 0;

    /**
     * Wait for a certain period of time to ensure the car has stopped,
     * don't make it turn while it's still moving, it will mess up the turn
     */
    ros::Duration(0.5).sleep();

    /**
     * Turn the car left by 90 degrees
     * The desired heading is the current heading + 90 degrees
     */
    float desired_heading = general_data->car_pose.th + (M_PI / 2.0); // Add 90 degrees (pi/2) to the current heading

    if (desired_heading > M_PI)
        desired_heading -= (2 * M_PI);
    else if (desired_heading < -M_PI)
        desired_heading += (2 * M_PI);

    general_data->car_target.x = general_data->car_pose.x;
    general_data->car_target.y = general_data->car_pose.y;
    general_data->car_target.th = desired_heading;

    RobotMovement(general_data);
}

void TurnCarRight90Degree(general_data_ptr general_data)
{
    // Stop the car
    general_data->car_vel.x = 0;
    general_data->car_vel.th = 0;

    /**
     * Wait for a certain period of time to ensure the car has stopped,
     * don't make it turn while it's still moving, it will mess up the turn
     */
    ros::Duration(0.5).sleep();

    /**
     * Turn the car right by 90 degrees
     * The desired heading is the current heading - 90 degrees
     */
    float desired_heading = general_data->car_pose.th - (M_PI / 2.0); // Subtract 90 degrees (pi/2) from the current heading

    if (desired_heading > M_PI)
        desired_heading -= (2 * M_PI);
    else if (desired_heading < -M_PI)
        desired_heading += (2 * M_PI);

    general_data->car_target.x = general_data->car_pose.x;
    general_data->car_target.y = general_data->car_pose.y;
    general_data->car_target.th = desired_heading;

    RobotMovement(general_data);
}

void KeepForward(general_data_ptr general_data)
{
    // Stop the car
    general_data->car_vel.x = 0;
    general_data->car_vel.th = 0;

    /**
     * Wait for a certain period of time to ensure the car has stopped,
     * don't make it turn while it's still moving, it will mess up the turn
     */
    ros::Duration(0.5).sleep();

    /**
     * Check if the robot's angle is linear with the angle of the lane first
     * If it is linear, keep moving forward
     * If it is not linear, adjust the target to align with the lane's angle
     */
    int middle_lane_size = general_data->middle_lane.size();
    int left_lane_size = general_data->left_lane.size();
    int right_lane_size = general_data->right_lane.size();
    int dist_between_points = general_data->middle_lane[middle_lane_size - 1].x - general_data->middle_lane[middle_lane_size - 5].x;

    int size_of_middle_lane_close_to_robot = SizeOfLane(general_data->middle_lane, 0, 30);

    if (size_of_middle_lane_close_to_robot == -1)
    {
        // keep forward
        general_data->car_target.x = general_data->car_pose.x + 0.5;
        general_data->car_target.y = general_data->car_pose.y;
        general_data->car_target.th = general_data->car_pose.th;
    }

    // Check if the robot's angle is linear with the angle of the lane
    if (middle_lane_size > 400 && abs(dist_between_points) < 20)
    {
        // The robot's angle is linear with the angle of the lane, keep moving forward
        general_data->car_target.x = general_data->car_pose.x;
        general_data->car_target.y = general_data->car_pose.y;
        general_data->car_target.th = general_data->car_pose.th;
    }
    else
    {
        // The robot's angle is not linear with the angle of the lane, adjust the target to align with the lane's angle
        if ((general_data->car_side == 10 && left_lane_size > 0) || (general_data->car_side == 20 && right_lane_size > 0))
        {
            general_data->car_target.x = (general_data->middle_lane_real[middle_lane_size - 1].x + general_data->car_pose.x) / 2;
            general_data->car_target.y = (general_data->middle_lane_real[middle_lane_size - 1].y + general_data->car_pose.y) / 2;
            general_data->car_target.th = atan2(general_data->car_target.y - general_data->car_pose.y, general_data->car_target.x - general_data->car_pose.x);
        }
        else
        {
            // The robot's angle is not linear and no specific side is determined, keep moving forward
            general_data->car_target.x = general_data->car_pose.x;
            general_data->car_target.y = general_data->car_pose.y;
            general_data->car_target.th = general_data->car_pose.th;
        }
    }

    // Call the RobotMovement function with the updated target to keep the car moving forward
    RobotMovement(general_data);
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
        float car_to_left = sqrt(pow(PixelToReal(700 - general_data->left_lane[left_lane_size - 1].y), 2) + pow(PixelToReal(400 - general_data->left_lane[left_lane_size - 1].x), 2));
        float car_to_right = sqrt(pow(PixelToReal(700 - general_data->right_lane[right_lane_size - 1].y), 2) + pow(PixelToReal(general_data->right_lane[right_lane_size - 1].x - 400), 2));
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

        // ROS_INFO("target %f %f %f\n", general_data->car_target.x, general_data->car_target.y, general_data->car_target.th);

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