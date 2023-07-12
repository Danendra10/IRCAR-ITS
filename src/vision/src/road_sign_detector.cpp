#include "sign_detector/aruco.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "road_sign_detector");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner spinner;
    image_transport::ImageTransport it(nh);

    if (ArucoInit() == -1)
    {
        ros::shutdown();
        return -1;
    }

    pub_detected_sign_data = nh.advertise<std_msgs::UInt16>("/vision/sign_detector/detected_sign_data", 1);
    pub_signal_stop = nh.advertise<std_msgs::UInt8>("/velocity/cmd/stop", 1);

    sub_raw_frame = it.subscribe("/catvehicle/camera_front/image_raw_front", 1, CallbackSubRawFrame);
    tim_30hz = nh.createTimer(ros::Duration(1.0 / 60.0), CallbackTimer30Hz);

    spinner.spin();
    return 0;
}

void CallbackSubRawFrame(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        mutex_raw_frame.lock();
        frame_raw = cv_bridge::toCvShare(msg, "bgr8")->image;
        mutex_raw_frame.unlock();
        validator |= 0b001;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void CallbackTimer30Hz(const ros::TimerEvent &event)
{
    if (validator != 0b001)
        return;
    cv::aruco::detectMarkers(frame_raw, dictionary, marker_corners, marker_ids, detector_params_ptr, rejected_candidates);

    cv::aruco::drawDetectedMarkers(frame_raw, marker_corners, marker_ids);
    // find the closest marker
    float min_dist = 1000000;
    int min_index = -1;
    for (int i = 0; i < marker_ids.size(); ++i)
    {
        int id = marker_ids[i];
        cv::Point2f center = (marker_corners[i][0] + marker_corners[i][1] + marker_corners[i][2] + marker_corners[i][3]) / 4;
        float dist = sqrt(center.x * center.x + center.y * center.y);
        if (dist < min_dist)
        {
            min_dist = dist;
            min_index = i;
        }
    }

    if (min_index == -1)
        counter++;
    else
        counter = 0;

    if (distance_to_detected_sign > distance_to_road_sign_threshold)
        stop_signal = 1;
    else
        stop_signal = 0;

    // printf("stop_signal: %d || Distance to detected sign: %f\n", stop_signal, distance_to_detected_sign);

    if (min_index != -1 && counter < threshold_counter_road_sign)
    {
        int id = marker_ids[min_index];
        cv::Point2f center = (marker_corners[min_index][0] + marker_corners[min_index][1] + marker_corners[min_index][2] + marker_corners[min_index][3]) / 4;
        float c_x = (marker_corners[min_index][0].x + marker_corners[min_index][1].x + marker_corners[min_index][2].x + marker_corners[min_index][3].x) / 4;
        float c_y = (marker_corners[min_index][0].y + marker_corners[min_index][1].y + marker_corners[min_index][2].y + marker_corners[min_index][3].y) / 4;
        distance_to_detected_sign = sqrt(pow(center.x - center_cam_x, 2) + pow(center.y - center_cam_y, 2));
        // printf("Detected point: (%f, %f) => Distance %f\n", center.x, center.y, distance_to_detected_sign);
        cv::putText(frame_raw, commands[id], center, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
        std_msgs::UInt16 msg;
        msg.data = id;
        pub_detected_sign_data.publish(msg);
    }
    else if (min_index == -1 && counter > threshold_counter_road_sign)
    {
        distance_to_detected_sign = 0;
        std_msgs::UInt16 msg;
        msg.data = 8;
        pub_detected_sign_data.publish(msg);
    }

    imshow("Raw Frame", frame_raw);

    std_msgs::UInt8 msg_signal;
    msg_signal.data = stop_signal;
    pub_signal_stop.publish(msg_signal);

    waitKey(1);
}

int ArucoInit()
{
    LoadConfig();
    detector_params = cv::aruco::DetectorParameters();
    detector_params_ptr = cv::makePtr<cv::aruco::DetectorParameters>(detector_params);
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    return 0;
}

void LoadConfig()
{
    try
    {
        char cfg_file[100];
        std::string current_path = ros::package::getPath("vision");
        sprintf(cfg_file, "%s/../../config/static_conf.yaml", current_path.c_str());

        printf("Loading config file: %s\n", cfg_file);

        YAML::Node config = YAML::LoadFile(cfg_file);
        threshold_counter_road_sign = config["threshold_counter_road_sign"].as<int>();
        distance_to_road_sign_threshold = config["distance_to_road_sign_threshold"].as<int>();
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
}