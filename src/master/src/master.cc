/**
 * Developed By @author
 * 1. @danendra10 github.com/danendra10
 * 2. @issabeljt github.com/issabeljt
 * 3. @hernanda16 github.com/hernanda16
 *
 * Auto Drive https://miro.com/app/board/uXjVM2AznZ0=/?share_link_id=411945080257
 */

#include "master/master.hh"

// #define DRIVE
#define TRANSMIT_VELOCITY

int main(int argc, char **argv)
{
    ros::init(argc, argv, "master");
    ros::NodeHandle NH;
    ros::MultiThreadedSpinner MTS;

    NH.getParam("is_urban", is_urban);

    if (MasterInit() == -1)
    {
        ros::shutdown();
        return -1;
    }

    general_instance.pub_car_vel = NH.advertise<geometry_msgs::Twist>("/catvehicle/cmd_vel_safe", 10);
    general_instance.pub_cmd_vision = NH.advertise<msg_collection::CmdVision>("/cmd_vision", 10);

    general_instance.sub_car_pose = NH.subscribe("/car_pose", 1, CllbckSubCarPose);
    general_instance.sub_real_lines = NH.subscribe("/real_lines", 1, CllbckSubRealLaneVector);
    general_instance.sub_lidar_data = NH.subscribe("/lidar_data", 1, CllbckSubLidarData);
    general_instance.sub_road_sign = NH.subscribe("/vision/sign_detector/detected_sign_data", 1, CllbckSubRoadSign);
    general_instance.sub_stop_signal = NH.subscribe<std_msgs::UInt8>("/velocity/cmd/stop", 1, boost::bind(CllbckSubSignalStop, _1, &general_instance));
    general_instance.sub_car_data = NH.subscribe<sensor_msgs::JointState>("/catvehicle/joint_states", 1, boost::bind(CllbckSubCarData, _1, &general_instance));
    general_instance.sub_vision_angle_error = NH.subscribe<std_msgs::Float32>("/vision/error_angle", 1, boost::bind(CllbckAngleError, _1, &general_instance));

    general_instance.tim_60_hz = NH.createTimer(ros::Duration(1 / 60), CllbckTim60Hz);

    MTS.spin();
    return 0;
}

void CllbckTim60Hz(const ros::TimerEvent &event)
{
    if (!is_urban)
    {
        GetKeyboard();
        SimulatorState();
        DecideCarTarget(&general_instance);
#ifdef TRANSMIT_VELOCITY
        TransmitData(&general_instance);
#endif
    }
    else
    {
        if (data_validator < 0b011)
            return;

        DriveUrban();
    }
}

void DriveUrban()
{
    static double start_time = ros::Time::now().toSec();
    static float current_angle = general_instance.car_pose.th;
    static float target_angle_right = current_angle - 90;
    static float target_angle_left = current_angle + 90;
    if (general_instance.prev_sign_type != general_instance.sign_type && (general_instance.prev_sign_type == -1 || general_instance.prev_sign_type == 8))
    {
        start_time = ros::Time::now().toSec();
        current_angle = general_instance.car_pose.th;
        // todo: butuh predefined target saja
        if (current_angle > 275 && current_angle < 45)
            target_angle_right = current_angle - 90;
        target_angle_left = current_angle + 90;
        general_instance.prev_sign_type = general_instance.sign_type;
    }
    // printf("general_instance.sign_type: %d %d\n", general_instance.sign_type, general_instance.prev_sign_type);
    if (general_instance.sign_type == SIGN_RIGHT)
    {
        printf("Time : %f\n", ros::Time::now().toSec() - start_time);
        if (target_angle_right < 0)
            target_angle_right += 360;
        else if (target_angle_right > 360)
            target_angle_right -= 360;

        float angle_error = general_instance.car_pose.th - target_angle_right;

        // if (angle_error > 360)
        //     angle_error -= 360;
        // else if (angle_error < 0)
        //     angle_error += 360;

        if ((ros::Time::now().toSec() - start_time) < ros::Duration(3).toSec())
        {
            printf("RIGHT\n");
            motion_return.linear = 3.5;
            motion_return.angular = 0;
            TransmitData(&general_instance);
            return;
        }

        printf("angle_error: %f || %f %f\n", angle_error, general_instance.car_pose.th, target_angle_right);
        if (fabs(angle_error) < 5)
        {
            general_instance.sign_type = NO_SIGN;
            goto withoutSign;
        }

        AngularControl(angle_error, 0.8);
        motion_return.linear = 3;

        TransmitData(&general_instance);
        return;
    }
    else if (general_instance.sign_type == SIGN_FORWARD)
    {
        if ((ros::Time::now().toSec() - start_time) < ros::Duration(5).toSec())
        {
            // printf("forward\n");
            motion_return.linear = 10;
            motion_return.angular = 0;
            TransmitData(&general_instance);
            return;
        }
        return;
    }
    else if (general_instance.sign_type == SIGN_LEFT)
    {
        if ((ros::Time::now().toSec() - start_time) < ros::Duration(3).toSec())
        {
            // printf("left\n");
            motion_return.linear = 10;
            TransmitData(&general_instance);
            return;
        }

        if (target_angle_left < 0)
            target_angle_left += 360;
        else if (target_angle_left > 360)
            target_angle_left -= 360;

        float angle_error = target_angle_left - general_instance.car_pose.th;

        if (angle_error > 180)
            angle_error -= 360;
        else if (angle_error < -180)
            angle_error += 360;

        if (fabs(angle_error) < 5)
        {
            general_instance.sign_type = NO_SIGN;
            goto withoutSign;
        }

        // printf("angle_error: %f || %f %f\n", angle_error, general_instance.car_pose.th, target_angle_left);
        AngularControl(angle_error, 0.5);
        motion_return.linear = 3;
        TransmitData(&general_instance);
        return;
    }
    else if (general_instance.sign_type == SIGN_STOP)
    {
        motion_return.linear = 0;
        motion_return.angular = 0;
        TransmitData(&general_instance);
        return;
    }
withoutSign:;
    DecideCarTarget(&general_instance);
    RobotMovement(&general_instance);
    TransmitData(&general_instance);

    general_instance.prev_sign_type = general_instance.sign_type;
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
    MotionControl(vx_, vz_);
}

void SimulatorState()
{
    switch (general_instance.main_state.value)
    {
    case FORWARD:
        MoveRobot(4, 0);
        break;

    case BACKWARD:
        MoveRobot(-4, 0);
        break;

    case TURN_LEFT:
        MoveRobot(3, 3);
        break;

    case TURN_RIGHT:
        MoveRobot(3, -3);
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
        if (data_validator < 0b111)
            return;

        static bool is_unlocked = true; // Initialize the locked state variable

        if (data->prev_sign_type != NO_SIGN && data->sign_type == NO_SIGN)
            is_unlocked = true;
        else
            is_unlocked = false;

        // Logger(RED, "is_unlocked: %d || Detected a sign %d", is_unlocked, data->sign_type);

        switch (data->sign_type)
        {
        case NO_SIGN:
            if (is_unlocked)
            {
                // Logger(RED, "No Sign Detected");
                // DecideCarTarget(data);
            }
            break;
        case SIGN_RIGHT:
            if (is_unlocked)
                is_unlocked = false;
            // Logger(RED, "Entered State Sign Right");
            if (TurnCarRight90Degree2(data, 0.5, 10) && !is_unlocked)
            {
                // Logger(RED, "Detected a Right Sign");
                is_unlocked = true;
                data->sign_type = NO_SIGN;
            }
            break;
        }

        data->prev_sign_type = data->sign_type;
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
    general_data->car_vel.x = 0.5;
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
            // SetRobotSteering(general_data, steering);
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
    // int middle_lane_size = general_data->middle_lane.size();
    // int left_lane_size = general_data->left_lane.size();
    // int right_lane_size = general_data->right_lane.size();
    // int dist_between_points = general_data->middle_lane[middle_lane_size - 1].x - general_data->middle_lane[middle_lane_size - 5].x;

    // int size_of_middle_lane_close_to_robot = SizeOfLane(general_data->middle_lane, 0, 30);

    // if (size_of_middle_lane_close_to_robot == -1) {
    //     // keep forward
    //     general_data->car_target_left.x = general_data->car_pose.x + 0.5;
    //     general_data->car_target_left.y = general_data->car_pose.y;
    //     general_data->car_target_left.th = general_data->car_pose.th;
    // }

    // // Check if the robot's angle is linear with the angle of the lane
    // if (middle_lane_size > 400 && abs(dist_between_points) < 20) {
    //     // The robot's angle is linear with the angle of the lane, keep moving forward
    //     general_data->car_target_left.x = general_data->car_pose.x;
    //     general_data->car_target_left.y = general_data->car_pose.y;
    //     general_data->car_target_left.th = general_data->car_pose.th;
    // } else {
    //     // The robot's angle is not linear with the angle of the lane, adjust the target to align with the lane's angle
    //     if ((general_data->car_side == 10 && left_lane_size > 0) || (general_data->car_side == 20 && right_lane_size > 0)) {
    //         general_data->car_target_left.x = (general_data->middle_lane_real[middle_lane_size - 1].x + general_data->car_pose.x) / 2;
    //         general_data->car_target_left.y = (general_data->middle_lane_real[middle_lane_size - 1].y + general_data->car_pose.y) / 2;
    //         general_data->car_target_left.th = atan2(general_data->car_target_left.y - general_data->car_pose.y, general_data->car_target_left.x - general_data->car_pose.x);
    //     } else {
    //         // The robot's angle is not linear and no specific side is determined, keep moving forward
    //         general_data->car_target_left.x = general_data->car_pose.x;
    //         general_data->car_target_left.y = general_data->car_pose.y;
    //         general_data->car_target_left.th = general_data->car_pose.th;
    //     }
    // }

    // Call the RobotMovement function with the updated target to keep the car moving forward
    RobotMovement(general_data);
}

void DecideCarTarget(general_data_ptr general_data)
{
    // left and right from car's pov
    general_data->car_target.x = general_data->buffer_target_x;
    general_data->car_target.y = general_data->buffer_target_y;

    // Logger(RED, "left %d || middle %d || right %d", general_data->left_available, general_data->middle_available, general_data->right_available);

    // Logger(MAGENTA, "%f - %f = %f", general_data->car_pose.x, general_data->prev_x_odom, general_data->car_pose.x - general_data->prev_x_odom);
    // Logger(RED, "target x : %f | target y : %f", general_data->car_target.x, general_data->car_target.y);
    try
    {
        if (data_validator < 0b011)
            return;

        float obs_from_left_target, obs_from_right_target;

        if (!general_data->obs_status)
            general_data->car_side = 0;

        else
        {
            float right_obs_y = general_data->raw_obs_data[0].y;
            float left_obs_y = general_data->raw_obs_data[general_data->raw_obs_data.size() - 1].y;

            if (general_data->left_available)
                obs_from_left_target = abs(left_obs_y - general_data->car_target_left.y);
            else if (general_data->middle_available)
                obs_from_left_target = abs(left_obs_y - (general_data->car_target_middle.y - 2 * general_data->divider));
            if (general_data->right_available)
                obs_from_right_target = abs(right_obs_y - general_data->car_target_right.y);
            else if (general_data->middle_available)
                obs_from_right_target = abs(right_obs_y - (general_data->car_target_middle.y + 2 * general_data->divider));
            // Logger(BLUE, "obs --- left %f right %f || dist --- left %f right %f", left_obs_y, right_obs_y, obs_from_left_target, obs_from_right_target);

            if ((obs_from_left_target - obs_from_right_target) > 0.05)
                general_data->car_side = 10;
            else if ((obs_from_right_target - obs_from_left_target) > 0.05)
                general_data->car_side = 20;
        }

        switch (general_data->car_side)
        {
        case 10:
            // printf("TARGET KIRI\n");
            if (general_data->left_available && general_data->middle_available)
            {
                Logger(CYAN, "KIRI TENGAH");
                general_data->car_target.x = lidar_range / 3.0;
                general_data->car_target.y = (general_data->car_target_left.y + general_data->car_target_middle.y) / 2.0 - general_data->spacer_real_y;
            }
            else if (general_data->left_available)
            {
                Logger(CYAN, "KIRI");
                general_data->car_target.x = lidar_range / 3.0;
                general_data->car_target.y = general_data->car_target_left.y + general_data->divider - general_data->spacer_real_y;
            }
            else if (general_data->middle_available)
            {
                Logger(CYAN, "TENGAH");
                general_data->car_target.x = lidar_range / 3.0;
                general_data->car_target.y = general_data->car_target_middle.y - general_data->divider - general_data->spacer_real_y;
            }

            break;

        case 20:
            // printf("TARGET KANAN\n");
            if (general_data->right_available && general_data->middle_available)
            {
                Logger(CYAN, "TENGAH KANAN");
                general_data->car_target.x = lidar_range / 3.0;
                general_data->car_target.y = (general_data->car_target_right.y + general_data->car_target_middle.y) / 2.0 + general_data->spacer_real_y;
            }
            else if (general_data->right_available)
            {
                Logger(CYAN, "KANAN");
                general_data->car_target.x = lidar_range / 3.0;
                general_data->car_target.y = general_data->car_target_right.y - general_data->divider + general_data->spacer_real_y;
            }
            else if (general_data->middle_available)
            {
                Logger(CYAN, "TENGAH");
                general_data->car_target.x = lidar_range / 3.0;
                general_data->car_target.y = general_data->car_target_middle.y + general_data->divider + general_data->spacer_real_y;
            }

            break;

        default:
            break;
        }

        // Logger(BLUE, "target x : %f | target y : %f", general_data->car_target.x, general_data->car_target.y);
        // Logger(YELLOW, "READYYYYYYY %d", general_instance.isReady);
        // ROS_INFO("FIXED TARGETTT %f %f\n", general_data->car_target.x, general_data->car_target.y);

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

    if (general_data->obs_status)
    {
        general_data->prev_x = general_data->car_pose.x;
        general_data->prev_y = general_data->car_pose.y;
        general_data->last_lidar_status = true;
    }
    else if (general_data->last_lidar_status && general_data->is_lidar_free)
    {
        Logger(MAGENTA, "KEEP FORWARD");
        general_data->car_target.y = 0;
        if (abs(sqrt(pow(general_data->prev_x, 2) + pow(general_data->prev_y, 2)) - sqrt(pow(general_data->car_pose.x, 2) + pow(general_data->car_pose.y, 2))) > 0.5)
        {
            general_data->last_lidar_status = false;
        }
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
    static float vel_linear = 0;
    static float vel_angular = 0;
    float ld = 10;

    // from rear wheel
    float dist_x = data->car_target.x + 3.8;
    float dist_y = data->car_target.y;

    float alpha = atan(dist_y / dist_x); // in rad
    if (dist_y < 0)
        alpha += DEG2RAD(180);
    float l = 2.8;
    float delta = atan(2 * l * sin(alpha) / ld);
    if (!is_urban)
    {
        vel_linear = 20;
        if (abs(data->car_target.y) > 4)
        {
            Logger(CYAN, "SLOWED DOWN");
            vel_linear *= 0.5;
        }
        else if (data->obs_status == true)
            vel_linear *= 0.65;
    }
    else
    {
        vel_linear = 10;
        if (data->obs_status == true)
            vel_linear *= 0.65;
    }

    if (data->car_target.y < 0)
        vel_angular = (delta)*vel_linear / 0.2;
    else
        vel_angular = -1 * (delta)*vel_linear / 0.2;

    MotionControl(vel_linear, vel_angular);
}

void TransmitData(general_data_ptr data)
{
    geometry_msgs::Twist vel_msg;
    float buffer_linear;
    float buffer_angular;
    if (linear_negative)
    {
        buffer_linear = -motion_return.linear;
    }
    else
    {
        buffer_linear = motion_return.linear;
    }

    if (angular_negative)
    {
        buffer_angular = -motion_return.angular;
    }
    else
    {
        buffer_angular = motion_return.angular;
    }
    vel_msg.linear.x = buffer_linear;
    vel_msg.angular.z = buffer_angular;
    data->pub_car_vel.publish(vel_msg);

    //============
    // command sent choose whether vision running standalone or being ordered master
    // some urgency make it happens. ex : there's obstacle but only got 2 lines of data making error in lane decision
    //============
    msg_collection::CmdVision cmd;
    cmd.find_3_lanes = false;
    data->pub_cmd_vision.publish(cmd);
}

//----------------------------------------------------------------------------------------------

int MasterInit()
{
    try
    {
        char cfg_file[100];
        std::string current_path = ros::package::getPath("vision");
        sprintf(cfg_file, "%s/../../config/static_conf.yaml", current_path.c_str());

        printf("Loading config file: %s\n", cfg_file);

        YAML::Node config = YAML::LoadFile(cfg_file);

        if (is_urban)
        {
            pid_linear_const.kp = config["PID_URBAN"]["Linear"]["kp"].as<float>();
            pid_linear_const.ki = config["PID_URBAN"]["Linear"]["ki"].as<float>();
            pid_linear_const.kd = config["PID_URBAN"]["Linear"]["kd"].as<float>();

            pid_angular_const.kp = config["PID_URBAN"]["Angular"]["kp"].as<float>();
            pid_angular_const.ki = config["PID_URBAN"]["Angular"]["ki"].as<float>();
            pid_angular_const.kd = config["PID_URBAN"]["Angular"]["kd"].as<float>();
        }
        else
        {
            pid_linear_const.kp = config["PID_RACE"]["Linear"]["kp"].as<float>();
            pid_linear_const.ki = config["PID_RACE"]["Linear"]["ki"].as<float>();
            pid_linear_const.kd = config["PID_RACE"]["Linear"]["kd"].as<float>();

            pid_angular_const.kp = config["PID_RACE"]["Angular"]["kp"].as<float>();
            pid_angular_const.ki = config["PID_RACE"]["Angular"]["ki"].as<float>();
            pid_angular_const.kd = config["PID_RACE"]["Angular"]["kd"].as<float>();
        }
    }
    catch (YAML::BadFile &e)
    {
        printf("Error: %s\n", e.what());
    }
    catch (YAML::ParserException &e)
    {
        printf("Error: %s\n", e.what());
    }
    catch (YAML::RepresentationException &e)
    {
        printf("Error: %s\n", e.what());
    }
    catch (std::exception &e)
    {
        printf("Error: %s\n", e.what());
    }
    catch (...)
    {
        cout << "Error caught on line: " << __LINE__ << endl;
    }
}