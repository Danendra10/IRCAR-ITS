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

    sub_raw_frame = it.subscribe("/catvehicle/camera_front/image_raw_front", 1, CallbackSubRawFrame);
    tim_30hz = nh.createTimer(ros::Duration(1.0 / 60.0), CallbackTimer30Hz);

    spinner.spin();
    return 0;
}

void CallbackSubRawFrame(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        frame_raw = cv_bridge::toCvShare(msg, "bgr8")->image;
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

    printf("Min index: %d\n", min_index);

    if (min_index == -1)
        counter++;
    else
        counter = 0;

    printf("Counter: %d\n", counter);

    if (min_index != -1 || counter < threshold_counter_road_sign)
    {
        int id = marker_ids[min_index];
        cv::Point2f center = (marker_corners[min_index][0] + marker_corners[min_index][1] + marker_corners[min_index][2] + marker_corners[min_index][3]) / 4;
        cv::putText(frame_raw, commands[id], center, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
        std_msgs::UInt16 msg;
        msg.data = id;
        printf("Published Data: %d\n", id);
        pub_detected_sign_data.publish(msg);
    }
    else if (min_index == -1 || counter > threshold_counter_road_sign)
    {
        std_msgs::UInt16 msg;
        msg.data = 8;
        printf("Published Data: 8\n");
        pub_detected_sign_data.publish(msg);
    }

    imshow("Raw Frame", frame_raw);
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