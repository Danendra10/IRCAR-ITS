#include "sign_detector/aruco.hpp"

#define SHOW_FRAME
// #define SHOW_SLIDER

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

    pub_detected_sign_data = nh.advertise<std_msgs::Int16>("/vision/sign_detector/detected_sign_data", 1);
    pub_signal_stop = nh.advertise<std_msgs::UInt8>("/velocity/cmd/stop", 1);

    sub_raw_frame = it.subscribe("/catvehicle/camera_front/image_raw_front", 1, CallbackSubRawFrame);
    tim_30hz = nh.createTimer(ros::Duration(1.0 / 30.0), CallbackTimer30Hz);

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

    if (frame_raw.empty())
    {
        ROS_ERROR("frame_raw is empty");
        return;
    }

    output_image = frame_raw.clone();

    //---Preprocessing
    /**
     * @brief I use thresholding to make the image black and white, then erode and dilate to remove noise
     * in this case it could decrease the noise from the raw image, i got better result with this method
     * if you came up with better method, please make a pull request and i'll review it.
     */
    cvtColor(frame_raw, frame_gray, CV_BGR2GRAY);
    threshold(frame_gray, thresholded, thresh_road_sign, max_val, THRESH_BINARY);
    // adaptiveThreshold(frame_gray, thresholded, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 11, 2);

    erode(thresholded, thresholded, Mat(), Point(-1, -1), 2);
    dilate(thresholded, thresholded, Mat(), Point(-1, -1), 2);

    //--->Aruco Detect marker using the thresholded image
    aruco::detectMarkers(frame_raw, dictionary, marker_corners, marker_ids, detector_params_ptr, rejected_candidates);
    aruco::drawDetectedMarkers(output_image, marker_corners, marker_ids);

    //---find the closest marker
    float min_dist = 1000000;
    int min_index = -1;
    static int prev_min_index = -1;
    static int last_id = -1;

    /**
     * @brief This for loop is used to find the closest marker to the camera
     * here we have the pre-defined minimum distance, if the distance of the marker is less than the minimum distance
     * then we update the minimum distance and the index of the marker
     */
    for (int i = 0; i < marker_ids.size(); ++i)
    {
        int id = marker_ids[i];
        Point2f center = (marker_corners[i][0] + marker_corners[i][1] + marker_corners[i][2] + marker_corners[i][3]) / 4;
        float dist = sqrt(center.x * center.x + center.y * center.y);
        if (dist < min_dist)
        {
            min_dist = dist;
            min_index = i;
        }
    }

    //---Safety check
    /**
     * @brief This is a safety check, if the minimum index is -1, then we increase the counter
     * minimum index return -1 if there is no marker detected, so we increase the counter
     * if the counter is greater than the threshold, then we assume that there is no marker detected
     * the threshold could be found in static_conf.yaml
     */
    if (min_index == -1)
        counter++;
    else
    {
        counter = 0;
        prev_min_index = min_index;
    }

    if (counter > threshold_to_delete_last_id)
    {
        last_id = -1;
    }

    static float area_of_marker;
    printf("Last ID: %d %f %d \n", last_id, area_of_marker, counter);

    // if (counter < threshold_counter_road_sign)
    // {
    // Get the id of the closest marker
    int id = marker_ids[prev_min_index];

    // Get the center of the marker in px
    Point2f center = (marker_corners[prev_min_index][0] + marker_corners[prev_min_index][1] + marker_corners[prev_min_index][2] + marker_corners[prev_min_index][3]) / 4;
    // For debug purpose
    putText(output_image, commands[id], center, FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);

    std_msgs::Int16 msg;
    /**
     * @brief This is a threshold to determine whether the marker is on the close spot
     * why i did not use the distance from the camera to the marker?
     * because the distance from the camera to the marker is not linear, so i use the position of the marker
     * you could find the threshold in static_conf.yaml if it match the condition it would publish the Id
     * to the master, if it's not then it would publish 8 to the master which means "no sign detected"
     */
    // printf("center.x: %f, center.y: %f || x_pos_road_sign_threshold: %d, y_pos_road_sign_threshold: %d\n", center.x, center.y, x_pos_road_sign_threshold, y_pos_road_sign_threshold);
    area_of_marker = (marker_corners[prev_min_index][0].x - marker_corners[prev_min_index][2].x) * (marker_corners[prev_min_index][0].y - marker_corners[prev_min_index][2].y);

    if (area_of_marker > 2000 && area_of_marker < 4000)
    {
        last_id = marker_ids[prev_min_index];
        msg.data = last_id;
    }
    else
        msg.data = last_id;
    pub_detected_sign_data.publish(msg);
    // }
    // else
    // {
    //     std_msgs::UInt16 msg;
    //     if (area_of_marker > 2550)
    //         msg.data = last_id;
    //     else
    //         msg.data = 8;
    //     pub_detected_sign_data.publish(msg);
    // }

#ifdef SHOW_FRAME
    // if (thresholded.empty() || frame_raw.empty() || output_image.empty())
    // {
    //     ROS_ERROR("thresholded is empty");
    //     return;
    // }
    // imshow("thresholded", thresholded);
    // imshow("Raw Frame", frame_raw);
    imshow("Out Frame", output_image);
#endif

    std_msgs::UInt8 msg_signal;
    msg_signal.data = stop_signal;
    pub_signal_stop.publish(msg_signal);

    waitKey(1);
}

int ArucoInit()
{
    LoadConfig();
    detector_params = aruco::DetectorParameters();
    detector_params_ptr = makePtr<aruco::DetectorParameters>(detector_params);
    dictionary = aruco::getPredefinedDictionary(aruco::DICT_APRILTAG_36h11);
#ifdef SHOW_SLIDER
    namedWindow("road_sign_thresholding", WINDOW_AUTOSIZE);
    createTrackbar("thresh", "road_sign_thresholding", &thresh_road_sign, 255);
    createTrackbar("max_val", "road_sign_thresholding", &max_val, 255);
#endif
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
        x_pos_road_sign_threshold = config["x_pos_road_sign_threshold"].as<int>();
        y_pos_road_sign_threshold = config["y_pos_road_sign_threshold"].as<int>();
        // thresh_road_sign = config["thresh_road_sign"].as<int>();
        // max_val = config["max_val"].as<int>();
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