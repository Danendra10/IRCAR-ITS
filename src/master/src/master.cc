/**
 * Developed By @author
 * 1. @danendra10 github.com/danendra10
 * 2. @issabeljt github.com/issabeljt
 * 3. @hernanda16 github.com/hernanda16
 *
 * Auto Drive https://miro.com/app/board/uXjVM2AznZ0=/?share_link_id=411945080257
 */

#include "master/master.hh"

#define DRIVE

int main(int argc, char **argv)
{
    ros::init(argc, argv, "master");
    ros::NodeHandle NH;
    ros::MultiThreadedSpinner MTS;

    general_instance.pub_car_vel = NH.advertise<geometry_msgs::Twist>("/catvehicle/cmd_vel_safe", 10);
    general_instance.pub_cmd_vision = NH.advertise<msg_collection::CmdVision>("/cmd_vision", 10);

    general_instance.sub_car_pose = NH.subscribe("/car_pose", 1, CllbckSubCarPose);
    /**
     * TODO: No data being published @hernanda16
     */
    general_instance.sub_lines = NH.subscribe("/lines", 1, CllbckSubLaneVector);
    general_instance.sub_real_lines = NH.subscribe("/real_lines", 1, CllbckSubRealLaneVector);
    general_instance.sub_lidar_data = NH.subscribe("/lidar_data", 1, CllbckSubLidarData);
    general_instance.sub_road_sign = NH.subscribe("/vision/sign_detector/detected_sign_data", 1, CllbckSubRoadSign);
    general_instance.sub_stop_signal = NH.subscribe<std_msgs::UInt8>("/velocity/cmd/stop", 1, boost::bind(CllbckSubSignalStop, _1, &general_instance));
    general_instance.sub_car_data = NH.subscribe<sensor_msgs::JointState>("/catvehicle/joint_states", 1, boost::bind(CllbckSubCarData, _1, &general_instance));

    general_instance.tim_60_hz = NH.createTimer(ros::Duration(1 / 60), CllbckTim60Hz);

    MTS.spin();
    return 0;
}

void CllbckTim60Hz(const ros::TimerEvent &event)
{
    // if (data_validator < 0b111)
    //     return;
    GetKeyboard();
    SimulatorState();
    // AutoDrive(&general_instance);
    // TurnCarRight90Degree2(&general_instance, -5, 10);
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

void AutoDrive(general_data_ptr data)
{
    try
    {
        if (data_validator < 0b1111)
        {
            return;
        }
        // Define a variable to store the previous sign type
        static int previous_sign_type = NO_SIGN;

        if (data->sign_type == NO_SIGN && previous_sign_type == NO_SIGN)
        {
            Logger(CYAN, "No Sign Detected");
            data->main_state.value = AUTONOMOUS_NO_SIGN;
        }
        else
        {
            // Check if the previous sign type is NO_SIGN
            if (previous_sign_type == NO_SIGN)
            {
                // Lock the current state based on the current sign type
                if (data->sign_type == SIGN_STOP)
                {
                    Logger(CYAN, "Detected a Stop Sign");
                    data->main_state.value = AUTONOMOUS_STOP_SIGN;
                }
                else if (data->sign_type == SIGN_LEFT)
                {
                    Logger(CYAN, "Detected a Left Sign");
                    data->main_state.value = AUTONOMOUS_TURN_LEFT_90;
                }
                else if (data->sign_type == SIGN_RIGHT)
                {
                    Logger(CYAN, "Detected a Right Sign");
                    data->main_state.value = AUTONOMOUS_TURN_RIGHT_90;
                }
                else if (data->sign_type == SIGN_FORWARD)
                {
                    Logger(CYAN, "Detected a Forward Sign");
                    data->main_state.value = AUTONOMOUS_KEEP_FORWARD;
                }
                // Uncomment and add additional sign types if necessary
            }
            printf("Previous and now %d %d\n", previous_sign_type, data->sign_type);
            // Update the previous sign type with the current sign type
            previous_sign_type = data->sign_type;
        }

#ifdef DRIVE
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
        default:
            RobotMovement(data);
            break;
        }
#endif
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    catch (...)
    {
        std::cout << "Error cought in line: " << __LINE__ << std::endl;
    }
}

void StopRobot(general_data_ptr data)
{
    data->car_vel.x = 0;
    data->car_vel.th = 0;
}

bool StopRobot(general_data_ptr data, float time_to_stop)
{
    static double time_1 = ros::Time::now().toSec();
    double current_time = ros::Time::now().toSec();

    if (current_time - time_1 < time_to_stop)
    {
        data->car_vel.x = 0;
        data->car_vel.th = 0;
    }
    else
    {
        return true;
    }
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

    general_data->car_target_left.x = general_data->car_pose.x;
    general_data->car_target_left.y = general_data->car_pose.y;
    general_data->car_target_left.th = desired_heading;

    RobotMovement(general_data);
}

void TurnCarRight90Degree(general_data_ptr general_data)
{
    // Stop the car
    // and duration of stop is more than 1 second
    // declare the stop time
    static double time_1 = ros::Time::now().toSec();
    double current_time = ros::Time::now().toSec();
    // double delta_time = current_time - stop_time;
    // printf("delta time: %f\n", delta_time);
    // if (general_data->signal_stop == 1 && delta_time < 1.0)
    // {
    //     general_data->car_vel.x = 0;
    //     general_data->car_vel.th = 0;

    //     /**
    //      * Wait for a certain period of time to ensure the car has stopped,
    //      * don't make it turn while it's still moving, it will mess up the turn
    //      */
    //     ros::Duration(0.5).sleep();
    //     return;
    // }

    /**
     * Turn the car right by 90 degrees
     * The desired heading is the current heading - 90 degrees
     */
    float desired_heading = general_data->car_pose.th + RAD2DEG(M_PI / 2.0); // Subtract 90 degrees (pi/2) from the current heading

    // desired heading is in degree
    if (desired_heading > 180)
        desired_heading -= 360;
    else if (desired_heading < -180)
        desired_heading += 360;

    desired_heading = DEG2RAD(desired_heading);

    general_data->car_vel.x = 1;
    general_data->car_vel.th = -0.5;

    // general_data->car_target_left.x = general_data->car_pose.x + 0.5 * cos(desired_heading);
    // general_data->car_target_left.y = general_data->car_pose.y + 0.5 * sin(desired_heading);
    // general_data->car_target_left.th = desired_heading;

    // general_data->car_target_left.x = general_data->car_pose.x;
    // general_data->car_target_left.y = general_data->car_pose.y;
    // general_data->car_target_left.th = desired_heading;

    float time_to_turn = 3.0;

    // UrbanMovement(general_data);
}

void SetRobotSteering(general_data_ptr general_data, float steering)
{
    general_data->car_vel.x = 1;
    general_data->car_vel.th = steering;
}

bool TurnCarRight90Degree2(general_data_ptr general_data, float steering, float time_to_turn)
{
    if (StopRobot(general_data, 1.0))
    {

        static double time_1 = ros::Time::now().toSec();
        double current_time = ros::Time::now().toSec();
        if (current_time - time_1 < time_to_turn)
        {
            printf("time: %f\n", current_time - time_1);
            SetRobotSteering(general_data, steering);
        }
        else
        {
            return true;
        }
    }
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
        general_data->car_target_left.x = general_data->car_pose.x + 0.5;
        general_data->car_target_left.y = general_data->car_pose.y;
        general_data->car_target_left.th = general_data->car_pose.th;
    }

    // Check if the robot's angle is linear with the angle of the lane
    if (middle_lane_size > 400 && abs(dist_between_points) < 20)
    {
        // The robot's angle is linear with the angle of the lane, keep moving forward
        general_data->car_target_left.x = general_data->car_pose.x;
        general_data->car_target_left.y = general_data->car_pose.y;
        general_data->car_target_left.th = general_data->car_pose.th;
    }
    else
    {
        // The robot's angle is not linear with the angle of the lane, adjust the target to align with the lane's angle
        if ((general_data->car_side == 10 && left_lane_size > 0) || (general_data->car_side == 20 && right_lane_size > 0))
        {
            general_data->car_target_left.x = (general_data->middle_lane_real[middle_lane_size - 1].x + general_data->car_pose.x) / 2;
            general_data->car_target_left.y = (general_data->middle_lane_real[middle_lane_size - 1].y + general_data->car_pose.y) / 2;
            general_data->car_target_left.th = atan2(general_data->car_target_left.y - general_data->car_pose.y, general_data->car_target_left.x - general_data->car_pose.x);
        }
        else
        {
            // The robot's angle is not linear and no specific side is determined, keep moving forward
            general_data->car_target_left.x = general_data->car_pose.x;
            general_data->car_target_left.y = general_data->car_pose.y;
            general_data->car_target_left.th = general_data->car_pose.th;
        }
    }

    // Call the RobotMovement function with the updated target to keep the car moving forward
    RobotMovement(general_data);
}

void DecideCarTarget(general_data_ptr general_data)
{
    try
    {
        if (data_validator < 0b0111)
            return;

        // left and right from car's pov
        float right_obs_y = general_data->raw_obs_data[0].y;
        float left_obs_y = general_data->raw_obs_data[general_data->raw_obs_data.size() - 1].y;
        float obs_from_left_target = abs(left_obs_y - general_data->car_target_left.y);
        float obs_from_right_target = abs(right_obs_y - general_data->car_target_right.y);
        // ROS_INFO("from left %f || from right %f\n", obs_from_left_target, obs_from_right_target);
        if (general_data->obs_status == 0)
            general_data->car_side = 0;
        else
        {
            if (obs_from_left_target > obs_from_right_target)
                general_data->car_side = 10;
            else if (obs_from_left_target < obs_from_right_target)
                general_data->car_side = 20;
        }
        switch (general_data->car_side)
        {
        case 10:
            printf("TARGET KIRI\n");
            general_data->car_target.x = general_data->car_target_left.x;
            general_data->car_target.y = general_data->car_target_left.y;
            break;

        case 20:
            printf("TARGET KANAN\n");
            general_data->car_target.x = general_data->car_target_right.x;
            general_data->car_target.y = general_data->car_target_right.y;
            break;

        default:
            printf("TENGAH\n");
            // general_data->car_target.x = general_data->car_target_left.x;
            // general_data->car_target.y = general_data->car_target_left.y;
            break;
        }

        ROS_INFO("FIXED TARGETTT %f %f\n", general_data->car_target.x, general_data->car_target.y);

        // ROS_INFO("target %f %f %f\n", general_data->car_target_left.x, general_data->car_target_left.y, general_data->car_target_left.th);
        // ROS_INFO("target %f %f %f\n", general_data->car_target_right.x, general_data->car_target_right.y, general_data->car_target_right.th);
        // if (general_data->middle_lane.size() < 0)
        //     return;
        // if (general_data->middle_lane[midd - 1].x == 0)
        //     return;
    }
    catch (const std::exception &e)
    {
        std::cout << "Error occurred on line: " << __LINE__ << std::endl;
        std::cout << "Error message: " << e.what() << std::endl;
    }
    catch (...)
    {
        ROS_ERROR_STREAM("Error caught on line: " << __LINE__);
    }
}

void UrbanMovement(general_data_ptr data)
{
    float ld = 10;
    float target_x = data->car_target.x;
    float target_y = data->car_target.y;
    float target_th = data->car_target.th;

    float dist_x = target_x + ld * cos(target_th);
    float dist_y = target_y + ld * sin(target_th);

    printf("target %f %f %f dist %f %f\n", target_x, target_y, target_th, dist_x, dist_y);
}

void RobotMovement(general_data_ptr data)
{
    // // PURE PURSUIT
    float ld = 10;
    // ROS_INFO("FIXED TARGETTT %f %f\n", data->car_target.x, data->car_target.y);

    // for (int i = 0; i < data->path_lane.size(); i++)
    // {
    //     float dist = sqrt(pow(data->path_lane[i].x, 2) + pow(data->path_lane[i].y, 2));
    //     // printf("i %d point %f %f dist %f diff %f\n", i, data->path_lane[i].x, data->path_lane[i].y, dist, abs(dist - ld));
    //     if (abs(dist - ld) < 0.01)
    //     {
    //         data->car_target.x = data->path_lane[i].x;
    //         data->car_target.y = data->path_lane[i].y;
    //         break;
    //     }
    // }

    // from rear wheel
    float dist_x = data->car_target.x + 3.8;
    float dist_y = data->car_target.y;

    float alpha = atan(dist_y / dist_x); // in rad
    if (dist_y < 0)
        alpha += DEG2RAD(180);
    float l = 2.8;

    float delta = atan(2 * l * sin(alpha) / ld);
    // printf("delta %f\n", delta);
    if (abs(delta) < 0.05)
    {
        data->car_vel.th = 0;
        data->car_vel.x = 2.8;
    }

    else
    {
        data->car_vel.x = 2.8;
        if (data->car_target.y < 0)
            data->car_vel.th = RAD2DEG(delta) / 16;
        // data->car_vel.th = RAD2DEG(delta) / (data->car_vel.x / 0.1);
        else
            // data->car_vel.th = -1 * RAD2DEG(delta) / (data->car_vel.x / 0.1);
            data->car_vel.th = -1 * RAD2DEG(delta) / 16;
    }
    printf("v th %f\n", data->car_vel.th);
    // printf("rear x %f y %f|| Car %f %f %f || wheel %f %f\n", rear_joint_x, rear_joint_y, data->car_pose.x, data->car_pose.y, data->car_pose.th, data->car_data.rear_left_wheel_joint, data->car_data.rear_right_wheel_joint);
}

void TransmitData(general_data_ptr data)
{
    geometry_msgs::Twist vel_msg;
    vel_msg.angular.z = data->car_vel.th;
    vel_msg.linear.x = data->car_vel.x;
    // if (general_instance.signal_stop)
    // {
    //     vel_msg.angular.z = 0;
    //     vel_msg.linear.x = 0;
    // }
    data->pub_car_vel.publish(vel_msg);

    //============
    // command sent choose whether vision running standalone or being ordered master
    // some urgency make it happens. ex : there's obstacle but only got 2 lines of data making error in lane decision
    //============
    msg_collection::CmdVision cmd;
    cmd.find_3_lanes = true;
    data->pub_cmd_vision.publish(cmd);
}