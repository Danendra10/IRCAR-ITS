/**
 * @author @Danendra10
 */

#include "master/master_urban.hh"

// #define DRIVE
#define TRANSMIT_VELOCITY

int main(int argc, char **argv)
{
    ros::init(argc, argv, "master_urban");
    ros::NodeHandle NH;
    ros::MultiThreadedSpinner MTS;

    if (MasterInit() == -1)
    {
        return -1;
    }

    general_instance.pub_car_vel = NH.advertise<geometry_msgs::Twist>("/catvehicle/cmd_vel_safe", 10);

    general_instance.sub_odom_data = NH.subscribe("/catvehicle/odom", 1, CllbckSubCarPose);
    general_instance.sub_lidar_data = NH.subscribe("/lidar_data", 1, CllbckSubLidarData);
    general_instance.sub_road_sign = NH.subscribe("/vision/sign_detector/detected_sign_data", 1, CllbckSubRoadSign);
    general_instance.sub_stop_signal = NH.subscribe<std_msgs::UInt8>("/velocity/cmd/stop", 1, boost::bind(CllbckSubSignalStop, _1, &general_instance));
    general_instance.sub_vision_road_sign_py = NH.subscribe<std_msgs::String>("/vision/road_sign_cmd", 1, boost::bind(CllbckSubVisionRoadSignPy, _1, &general_instance));
    general_instance.sub_vision_angle_error = NH.subscribe<std_msgs::Float32>("/vision/error_angle", 1, boost::bind(CllbckAngleError, _1, &general_instance));

    general_instance.tim_60_hz = NH.createTimer(ros::Duration(1.0 / 60.0), CllbckTim60Hz);

    MTS.spin();
}

void TransmitAll()
{
    geometry_msgs::Twist msg;
    msg.linear.x = motion_return.linear;
    msg.angular.z = motion_return.angular;
    general_instance.pub_car_vel.publish(msg);
}

void CllbckTim60Hz(const ros::TimerEvent &event)
{
    if (general_instance.sign_type == SIGN_RIGHT)
    {
        static float current_angle = general_instance.car_pose.th;
        static float target_angle;
        if (current_angle > 270 || current_angle < 90)
        {
            target_angle = 270;
        }
        else
        {
            target_angle = 180;
        }
        float angle_error = target_angle - general_instance.car_pose.th;
        printf("angle_error: %f || %f %f\n", angle_error, general_instance.car_pose.th, target_angle);
        AngularControl(angle_error, 0.2);
        motion_return.linear = 2;
        TransmitAll();
        return;
    }
    else if (general_instance.sign_type == SIGN_LEFT)
    {
        static float current_angle = general_instance.car_pose.th;
        static float target_angle;
        if (current_angle > 270 || current_angle < 90)
        {
            target_angle = 90;
        }
        else
        {
            target_angle = 180;
        }
        float angle_err = target_angle - general_instance.car_pose.th;
        AngularControl(angle_err, 0.2);
        motion_return.linear = 2;
        TransmitAll();
        return;
    }
    if (general_instance.angle_error != 0)
    {
        AngularControl(general_instance.angle_error, 1);
        motion_return.linear = 1;
        cout << "linear: " << motion_return.linear << endl;
        cout << "angular: " << motion_return.angular << endl;
        TransmitAll();
    }
}

int MasterInit()
{
    try
    {
        char cfg_file[100];
        std::string current_path = ros::package::getPath("vision");
        sprintf(cfg_file, "%s/../../config/static_conf.yaml", current_path.c_str());

        printf("Loading config file: %s\n", cfg_file);

        YAML::Node config = YAML::LoadFile(cfg_file);

        pid_linear_const.kp = config["PID"]["Linear"]["kp"].as<float>();
        pid_linear_const.ki = config["PID"]["Linear"]["ki"].as<float>();
        pid_linear_const.kd = config["PID"]["Linear"]["kd"].as<float>();

        pid_angular_const.kp = config["PID"]["Angular"]["kp"].as<float>();
        pid_angular_const.ki = config["PID"]["Angular"]["ki"].as<float>();
        pid_angular_const.kd = config["PID"]["Angular"]["kd"].as<float>();
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