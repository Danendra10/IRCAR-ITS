/**
 * @copyright @Danendra10 & @isabellejt & @hernanda16
 * @brief This node will calculate the line of the road
 *       and publish the point of the line
 * @license IRIS
 */

#include "vision/vision.hh"
int a, b, c;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision");
    ros::NodeHandle NH;
    image_transport::ImageTransport IT(NH);
    ros::MultiThreadedSpinner MTS(0);

    // namedWindow("thresholding", WINDOW_AUTOSIZE);
    // createTrackbar("Thresh", "thresholding", &a, 200);
    // createTrackbar("Min Length", "thresholding", &b, 150);
    // createTrackbar("Max Gap", "thresholding", &c, 50);

    Init();
    LogParams();

    NH.getParam("is_urban", is_urban);
    tim_30hz = NH.createTimer(ros::Duration(1.0 / 30.0), Tim30HzCllbck);
    sub_raw_frame = IT.subscribe("/catvehicle/camera_front/image_raw_front", 1, SubRawFrameCllbck);
    sub_odom = NH.subscribe("/catvehicle/odom", 1, SubOdomRaw);
    sub_lidar_data = NH.subscribe("/lidar_data", 1, SubLidarData);
    sub_cmd_vision = NH.subscribe("/cmd_vision", 1, SubCmdVision);

    pub_car_pose = NH.advertise<geometry_msgs::Point>("/car_pose", 1);
    pub_target = NH.advertise<msg_collection::RealPosition>("/real_lines", 1);

    MTS.spin();

    return 0;
}

void SubCmdVision(const msg_collection::CmdVision::ConstPtr &msg)
{
    find3Lines = msg->find_3_lanes;
}

void SubRawFrameCllbck(const sensor_msgs::ImageConstPtr &msg)
{
    mutex_raw_frame.lock();
    raw_frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    mutex_raw_frame.unlock();

    validators |= 0b001;
}

void SubOdomRaw(const nav_msgs::Odometry::ConstPtr &msg)
{
    car_pose.x = msg->pose.pose.position.x;
    car_pose.y = msg->pose.pose.position.y;

    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;

    double roll, pitch, yaw;

    double t0 = +2.0 * (qw * qx + qy * qz);
    double t1 = +1.0 - 2.0 * (qx * qx + qy * qy);
    roll = std::atan2(t0, t1);

    double t2 = +2.0 * (qw * qy - qz * qx);
    t2 = (t2 > 1.0) ? 1.0 : t2;
    t2 = (t2 < -1.0) ? -1.0 : t2;
    pitch = std::asin(t2);

    double t3 = +2.0 * (qw * qz + qx * qy);
    double t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
    yaw = std::atan2(t3, t4);

    double heading = yaw;

    double heading_deg = std::fmod(RAD2DEG(heading) + 360.0, 360.0);
    printf("heading: %f\n", heading_deg);

    // car_pose.th = RAD2DEG(2 * atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));

    geometry_msgs::Point car_pose_msg;
    car_pose_msg.x = car_pose.x;
    car_pose_msg.y = car_pose.y;
    car_pose_msg.z = car_pose.th;
    pub_car_pose.publish(car_pose_msg);
}

void SubLidarData(const msg_collection::Obstacles::ConstPtr &msg)
{
    obstacles.clear();
    raw_obstacles.clear();
    for (int i = 0; i < msg->x.size(); i++)
    {
        ObstaclesPtr obstacle(new Obstacles);
        obstacle->x = msg->x[i] + car_pose.x;
        obstacle->y = msg->y[i] + car_pose.y;
        obstacles.push_back(obstacle);

        ObstaclesPtr raw_obstacle(new Obstacles);
        raw_obstacle->x = msg->x[i];
        raw_obstacle->y = msg->y[i];
        raw_obstacle->dist = msg->dist[i];
        raw_obstacles.push_back(raw_obstacle);
    }
}

void Tim30HzCllbck(const ros::TimerEvent &event)
{
    if (validators != 0b001)
        return;

    Mat frame_resize;
    Mat frame_gray_resize;
    Mat frame_remapped = Mat(DST_REMAPPED_HEIGHT, DST_REMAPPED_WIDTH, CV_8UC1);

    resize(raw_frame, frame_resize, Size(cam_params.image_width, cam_params.image_height));
    cvtColor(frame_resize, frame_gray_resize, COLOR_BGR2GRAY);

    InversePerspective(DST_REMAPPED_WIDTH, DST_REMAPPED_HEIGHT, frame_gray_resize.data, maptable, frame_remapped.data);

    Mat line_bgr;
    cvtColor(frame_remapped, line_bgr, COLOR_GRAY2BGR);

    Detect(line_bgr);

    waitKey(1);
}

//========================================================================================================================

void click_event(int event, int x, int y, int flags, void *params)
{
    if (event == EVENT_LBUTTONDOWN)
    {
        float distance_on_frame = sqrt(pow((x - 400), 2) + pow((800 - y), 2));
        printf("CLICKED x %d y %d dist %f\n\n", x - 400, 800 - y, distance_on_frame);
    }
}
void record()
{
    int frame_width = 800;
    int frame_height = 800;

    VideoWriter video;

    video.open("/home/isabellej/Desktop/nihh.mp4", VideoWriter::fourcc('m', 'p', '4', 'v'), 10, Size(frame_width, frame_height));
    for (int i = 0; i < 7000; i++)
    {
        video.write(raw_frame);
        printf("%d recording.\n", i);
    }
    video.release();
    destroyAllWindows();
}

void Init()
{
    curr_time = std::chrono::system_clock::now();
    cam_params.horizontal_fov = 1.3962634; // rad
    cam_params.vertical_fov = 2 * atan(tan(cam_params.horizontal_fov / 2) * 1);
    cam_params.image_width = 800;
    cam_params.image_height = 800;
    cam_params.near_clip = 0.1;
    cam_params.near_clip = 0.02;
    cam_params.far_clip = 300;
    cam_params.noise_mean = 0.0;
    cam_params.noise_std_dev = 0.007;
    cam_params.hack_baseline = 0.07;
    cam_params.distortion_k1 = 0.0;
    cam_params.distortion_k2 = 0.0;
    cam_params.distortion_k3 = 0.0;
    cam_params.distortion_t1 = 0.0;
    cam_params.distortion_t2 = 0.0;
    cam_params.camera_pos_x = 75; // cm
    cam_params.camera_pos_y = 0;
    cam_params.camera_pos_z = 202.5;
    cam_params.cam_scale_x = (2 * cam_params.camera_pos_x * tan(cam_params.horizontal_fov / 2)) / cam_params.image_width;
    cam_params.cam_scale_y = (2 * cam_params.camera_pos_y * tan(cam_params.vertical_fov / 2)) / cam_params.image_height;
    printf("hfov %f\n", cam_params.vertical_fov);
    BuildIPMTable(SRC_RESIZED_WIDTH, SRC_RESIZED_HEIGHT, DST_REMAPPED_WIDTH, DST_REMAPPED_HEIGHT, DST_REMAPPED_WIDTH >> 1, DST_REMAPPED_HEIGHT >> 1, maptable);
}

void Detect(cv::Mat frame)
{
    // Logger(MAGENTA, "Detecting ...");
    cv::Mat frame_gray, frame_canny, frame_thresh, frame_road_thresh, frame_side_thresh;
    cv::Mat result = frame.clone();

    cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(frame_gray, frame_gray, cv::Size(5, 5), 0);

#ifndef edge_detection
    //==Option Threshold
    if (is_urban)
    {
        cv::threshold(frame_gray, frame_thresh, 130, 255, cv::THRESH_BINARY);          // for line
        cv::threshold(frame_gray, frame_side_thresh, 5, 255, cv::THRESH_BINARY);       // for black side of IPM
        cv::threshold(frame_gray, frame_road_thresh, 130, 255, cv::THRESH_BINARY_INV); // for road
        cv::bitwise_and(frame_side_thresh, frame_road_thresh, frame_road_thresh);
        cv::erode(frame_thresh, frame_thresh, Mat(), Point(-1, -1), 1);
        cv::dilate(frame_thresh, frame_thresh, Mat(), Point(-1, -1), 5);
        cv::erode(frame_thresh, frame_thresh, Mat(), Point(-1, -1), 4);
        cv::erode(frame_road_thresh, frame_road_thresh, Mat(), Point(-1, -1), 2);
        cv::dilate(frame_road_thresh, frame_road_thresh, Mat(), Point(-1, -1), 8);
        cv::erode(frame_road_thresh, frame_road_thresh, Mat(), Point(-1, -1), 7);
        cv::bitwise_and(frame_road_thresh, frame_thresh, frame_thresh);
        // if (!frame_thresh.empty())
        // cv::imshow("line thresh", frame_thresh);
        // cv::imshow("side thresh", frame_side_thresh);
        // if (!frame_road_thresh.empty())
        // cv::imshow("road thresh", frame_road_thresh);
    }
    else
    {
        cv::threshold(frame_gray, frame_thresh, 55, 255, cv::THRESH_BINARY);
    }
    erode(frame_thresh, frame_thresh, Mat(), Point(-1, -1), 1);
    dilate(frame_thresh, frame_thresh, Mat(), Point(-1, -1), 1);
    ROI(frame_thresh);
    if (is_urban && !frame_thresh.empty())
    // cv::imshow("threshold ROI", frame_thresh);
#endif

#ifdef edge_detection
        //==Option Edge
        cv::Canny(frame_gray, frame_canny, 25, 50);
    dilate(frame_canny, frame_canny, Mat(), Point(-1, -1), 6);
    erode(frame_canny, frame_canny, Mat(), Point(-1, -1), 4);
    ROI(frame_canny);
#endif

    if (!frame_thresh.empty())
    {
        BinaryStacking(frame_thresh, result);
    }
    else if (!frame_canny.empty())
    {
        BinaryStacking(frame_canny, result);
    }

    cv::Mat frame_thresh_resized;
    cv::Mat frame_canny_resized;
    cv::Mat result_resized;

    // Logger(CYAN, "Showing image");
    if (!frame_thresh.empty())
    {
        // Logger(CYAN, "Showing thresh in result");
        cv::resize(frame_thresh, frame_thresh_resized, cv::Size(400, 400));
        cv::cvtColor(frame_thresh, frame_thresh, cv::COLOR_GRAY2BGR);
        cv::addWeighted(frame_thresh, 0.3, result, 1.0, 0.0, result);
    }
    if (!frame_canny.empty())
    {
        // Logger(CYAN, "Showing canny in result");
        cv::resize(frame_canny, frame_canny_resized, cv::Size(400, 400));
        cv::cvtColor(frame_canny, frame_canny, cv::COLOR_GRAY2BGR);
        cv::addWeighted(frame_canny, 0.3, result, 1.0, 0.0, result);
    }
    if (!result.empty())
    {
        // Logger(CYAN, "Showing result");
        cv::resize(result, result_resized, cv::Size(400, 400));
        // cv::imshow("result", result_resized);
    }
}

void ROI(cv::Mat &frame)
{
    // Logger(CYAN, "Finding ROI");
    cv::Mat frame_mask(frame.rows, frame.cols, CV_8UC1, cv::Scalar(0));
    std::vector<cv::Point> ROI;

    if (is_urban)
    {
        ROI.push_back(cv::Point(270, 750)); // bottom left
        ROI.push_back(cv::Point(530, 750)); // bottom right
        ROI.push_back(cv::Point(600, 670));
        ROI.push_back(cv::Point(600, 500)); // top right
        ROI.push_back(cv::Point(200, 500)); // top left
        ROI.push_back(cv::Point(200, 670));
    }
    else
    {
        ROI.push_back(cv::Point(270, 750)); // bottom left
        ROI.push_back(cv::Point(530, 750)); // bottom right
        ROI.push_back(cv::Point(700, 550));
        ROI.push_back(cv::Point(700, 500)); // top right
        ROI.push_back(cv::Point(100, 500)); // top left
        ROI.push_back(cv::Point(100, 550));
    }

    std::vector<cv::Point> ROI_left;

    ROI_left.push_back(cv::Point(500, 700)); // bottom left
    ROI_left.push_back(cv::Point(650, 700)); // bottom right
    ROI_left.push_back(cv::Point(650, 440)); // top right
    ROI_left.push_back(cv::Point(500, 440)); // top left

    std::vector<cv::Point> ROI_right;

    ROI_right.push_back(cv::Point(300, 700)); // bottom right
    ROI_right.push_back(cv::Point(150, 700)); // bottom left
    ROI_right.push_back(cv::Point(150, 440)); // top left
    ROI_right.push_back(cv::Point(300, 440)); // top right

    std::vector<cv::Point> ROI_center;

    ROI_center.push_back(cv::Point(320, 750)); // bottom left
    ROI_center.push_back(cv::Point(480, 750)); // bottom right
    ROI_center.push_back(cv::Point(480, 500)); // top right
    ROI_center.push_back(cv::Point(320, 500)); // top left

    fillConvexPoly(frame_mask, ROI, cv::Scalar(255));
    cv::bitwise_and(frame, frame_mask, frame);

    // cv::imshow("mask roi", frame_mask);
}

void Display(cv::Mat &frame, std::vector<cv::Vec4i> lines, int b_, int g_, int r_, float intensity)
{
    cv::Mat draw(frame.rows, frame.cols, CV_8UC3, cv::Scalar(0));
    if (lines.size() > 0)
        for (size_t i = 0; i < lines.size(); i++)
        {
            cv::Vec4i line = lines[i];
            cv::line(draw, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(b_, g_, r_), 10);
        }

    cv::addWeighted(draw, intensity, frame, 1.0, 0.0, frame);

    // cv::imshow("line", draw);
}

std::vector<cv::Vec4i> SlidingWindows(cv::Mat &frame, std::vector<int> x_final, std::vector<int> nonzero_x, std::vector<int> nonzero_y)
{
    // Logger(CYAN, "Entering Sliding Windows Calculation");
    std::vector<cv::Rect> windows;
    const int num_windows = 9;
    const int margin = 50;
    const int min_px = 100;
    int window_height = (int)((750 - 500) / (float)num_windows); // from ROI
    std::vector<int> current_x;
    std::vector<std::vector<int>> in_window(500);
    cv::Vec2i win_y;

    for (int i = 0; i < x_final.size(); i++)
    {
        current_x.push_back(x_final[i]);
        // Logger(RED, "current_x : %d", current_x[i]);
    }

    std::vector<cv::Vec2i> win_x(current_x.size());
    for (int i = 0; i < num_windows; i++)
    {
        win_y[0] = 750 - (i + 1) * window_height; // from ROI
        win_y[1] = 750 - i * window_height;       // from ROI
        for (int j = 0; j < current_x.size(); j++)
        {
            win_x[j][0] = current_x[j] - margin;
            win_x[j][1] = current_x[j] + margin;
            // std::cout << win_x[j] << std::endl;
            cv::rectangle(frame, cv::Rect(win_x[j][0], win_y[0], win_x[j][1] - win_x[j][0], win_y[1] - win_y[0]), cv::Scalar(255, 0, 0), 2);

            for (int k = 0; k < nonzero_x.size(); k++)
            {
                if (nonzero_x[k] >= win_x[j][0] && nonzero_x[k] < win_x[j][1] && nonzero_y[k] >= win_y[0] && nonzero_y[k] < win_y[1])
                {
                    in_window[j].push_back(k);
                }
            }

            if (in_window[j].size() > 100)
            {
                int min_y = 800;
                int max_y = 0;
                int top_x;
                int bottom_x;
                int temp = win_y[1];
                int sum = 0;

                for (int k = 0; k < in_window[j].size(); k++)
                {
                    if (nonzero_y[in_window[j][k]] > max_y)
                    {
                        bottom_x = nonzero_x[in_window[j][k]];
                    }
                    if (nonzero_y[in_window[j][k]] < min_y)
                    {
                        top_x = nonzero_x[in_window[j][k]];
                    }
                }
                current_x[j] = top_x;

                // current_x[j] = (int)((max + min) / 2.0);
                // cv::circle(frame, cv::Point(current_x[j], win_y[0]), 3, cv::Scalar(0, 0, 255), 10);
            }
        }
        // std::cout << win_y << std::endl;
    }

    std::vector<cv::Vec4i> in_points(current_x.size());

    for (int i = 0; i < current_x.size(); i++)
    {
        int temp_y_max = 0;
        int temp_y_min = 800;
        for (int j = 0; j < in_window[i].size(); j++)
        {
            // in_points[i][0] = nonzero_x[in_window[i][j]];
            // in_points[i][1] = nonzero_y[in_window[i][j]];
            // cv::circle(frame, cv::Point(in_points[i][0], in_points[i][1]), 1, cv::Scalar((i + 1) * 80, 0, 0), 10);
            if (nonzero_y[in_window[i][j]] < temp_y_min)
            {
                temp_y_min = nonzero_y[in_window[i][j]];
                in_points[i][0] = nonzero_x[in_window[i][j]];
            }
            if (nonzero_y[in_window[i][j]] > temp_y_max)
            {
                temp_y_max = nonzero_y[in_window[i][j]];
                in_points[i][2] = nonzero_x[in_window[i][j]];
            }
        }
        in_points[i][1] = temp_y_min;
        in_points[i][3] = temp_y_max;
    }
    // Logger(CYAN, "Sliding windows equation finished");
    return in_points;
}

/**
 * This function is used to detect line by converting 800x800 to 1x800 and every non-zero value considered as lines
 */
void BinaryStacking(cv::Mat frame, cv::Mat &frame_dst)
{
    mutex_binary_frame.lock();
    cv::Mat frame_binary = frame.clone();
    mutex_binary_frame.unlock();

    // Logger(CYAN, "Processing Binary Stacking");
    std::vector<cv::Vec2i> nonzero;
    cv::findNonZero(frame_binary, nonzero);

    std::vector<int> nonzero_y(nonzero.size());
    std::vector<int> nonzero_x(nonzero.size());

    /**
     * This for loop is used to separate non-zero array from frame(only non-zero's index is stored)
     */
    for (int i = 0; i < nonzero.size(); i++)
    {
        nonzero_x[i] = nonzero[i][0];
        nonzero_y[i] = nonzero[i][1];
    }

    cv::Mat verticalSum;
    cv::Mat binaryMask = (frame_binary > 0) / 255;
    cv::reduce(binaryMask, verticalSum, 0, cv::REDUCE_SUM, CV_32S);

    // std::cout << verticalSum << std::endl;
    if (binaryMask.empty())
    {
        // Logger(RED, "binaryMask is empty");
    }
    if (verticalSum.empty())
    {
        // Logger(RED, "verticalSum is empty");
    }

    const int mid_point = verticalSum.cols / 2.0;
    bool isZero = true;
    bool prev_isZero = true;
    int spike = 0;
    int spike_final;
    std::vector<int> start;
    std::vector<int> stop;
    std::vector<int> center_x_base;
    std::vector<int> center_x_final;
    int counter = 0;

    for (size_t i = 0; i < verticalSum.cols; i++)
    {
        prev_isZero = isZero;
        if (verticalSum.at<int>(0, i) > 0 && prev_isZero)
        {
            isZero = false;
            start.push_back(i);
            spike++;
        }
        else if (verticalSum.at<int>(0, i) == 0 && !prev_isZero)
        {
            isZero = true;
            stop.push_back(i);
        }
    }

    for (int i = 0; i < spike; i++)
    {
        // Logger(YELLOW, "start : %d || stop : %d", start[i], stop[i]);
    }

    // Logger(RED, "spike : %d", spike);

    center_x_base.resize(spike);

    for (int i = 0; i < spike; i++)
    {
        CenterSpike(verticalSum, start[i], stop[i], center_x_base[i]);
        // Logger(BLUE, "center x[%d] : %d", i, center_x_base[i]);
        if (center_x_base[i] - center_x_base[i - 1] < 100 && i != 0)
        {
            center_x_final.pop_back();
            center_x_final.push_back((center_x_base[i] + center_x_base[i - 1]) / 2.0);
            counter++;
        }
        else
        {
            center_x_final.push_back(center_x_base[i]);
        }

        //==Debug Sliding Windows Coverage with Noise
        // cv::circle(frame_dst, cv::Point(center_x_base[i], 720), 3, cv::Scalar(255, 0, 0), 10);
        // cv::circle(frame_dst, cv::Point(center_x_base[i], 460), 3, cv::Scalar(255, 0, 0), 10);
    }

    // Logger(CYAN, "spike : %d | counter : %d", spike, counter);

    spike_final = spike - counter;

    //==Debug Sliding Windows Coverage Final
    // for (int i = 0; i < spike_final; i++) {
    //     cv::circle(frame_dst, cv::Point(center_x_final[i], 700), 3, cv::Scalar(i * 50, 255, 0), 10);
    //     cv::circle(frame_dst, cv::Point(center_x_final[i], 440), 3, cv::Scalar(i * 50, 255, 0), 10);
    // }

    std::vector<cv::Vec4i> in_points = SlidingWindows(frame_dst, center_x_final, nonzero_x, nonzero_y);
    std::vector<std::vector<cv::Vec4i>> lane(in_points.size());

    // just for drawing
    // for (int i = 0; i < in_points.size(); i++)
    // {
    //     lane[i].push_back(in_points[i]);
    //     // Display(frame_dst, lane[i], i * 100, 1 * 100, 255, 0.5);
    //     // SlopeIntercept(in_points[i], line_SI[i][0], line_SI[i][1]); //0 -> left, 1-> middle, 2 -> right
    //     // cv::circle(frame_dst, cv::Point((int)((frame_binary.rows - 220 - line_SI[i][1]) / line_SI[i][0]), frame_binary.rows - 220), 3, cv::Scalar(i * 100, i * 100, 255), 7);
    // }

    for (int i = 0; i < 3; i++)
    {
        line_SI[i][0] = 0;
        line_SI[i][1] = 0;
    }

    prev_left[0] = 6969;
    prev_middle[0] = 6969;
    prev_right[0] = 6969;

    // problem is used to debug if road target has problem not being able to identify new road target from prev x in new sliding windows
    problem = true;

    // Logger(CYAN, "Finding n-lines");
    if (spike_final == 3)
    {
        // Logger(GREEN, "3 LINES DETECTED");
        road_target = 1;
        must3Lines = false;
        canbeIntercept = true;
        problem = false;

        y_target = frame_binary.rows - 200;
        x_target = (in_points[road_target][0] + in_points[road_target][2]) / 2.0;

        prev_left[0] = 0;
        prev_middle[0] = 1;
        prev_right[0] = 2;

        prev_left[1] = (in_points[prev_left[0]][0] + in_points[prev_left[0]][2]) / 2.0;
        prev_middle[1] = (in_points[prev_middle[0]][0] + in_points[prev_middle[0]][2]) / 2.0;
        prev_right[1] = (in_points[prev_right[0]][0] + in_points[prev_right[0]][2]) / 2.0;
    }
    else if (spike_final > 3 && !must3Lines)
    {
        for (int i = 0; i < in_points.size(); i++)
        {
            // checking if prev x is in one of new sliding windows then assign new index of sliding windows become road target(middle)
            if (abs(prev_x_target - in_points[i][0]) < 100 && abs(prev_x_target - in_points[i][2]) < 100)
            {
                // Logger(GREEN, "MORE THAN 3 LINES DETECTED");
                road_target = i;
                // Logger(YELLOW, "New Road Target %d", i);
                problem = false;
            }

            if (abs(prev_left[1] - in_points[i][0]) <= 100 && abs(prev_left[1] - in_points[i][2]) <= 100)
            {
                prev_left[0] = i;
                prev_left[1] = (in_points[prev_left[0]][0] + in_points[prev_left[0]][2]) / 2.0;
            }
            else if (abs(prev_middle[1] - in_points[i][0]) <= 100 && abs(prev_middle[1] - in_points[i][2]) <= 100)
            {
                prev_middle[0] = i;
                prev_middle[1] = (in_points[prev_middle[0]][0] + in_points[prev_middle[0]][2]) / 2.0;
            }
            else if (abs(prev_right[1] - in_points[i][0]) <= 100 && abs(prev_right[1] - in_points[i][2]) <= 100)
            {
                prev_right[0] = i;
                prev_right[1] = (in_points[prev_right[0]][0] + in_points[prev_right[0]][2]) / 2.0;
            }
        }

        // Logger(GREEN, "MORE THAN 3 |==| left : %d | mid : %d | right : %d", prev_left[0], prev_middle[0], prev_right[0]);

        // if above cannot determine new road target from prev x in new sliding windows, problem cannot be false
        if (problem)
        {
            // Logger(RED, "SOMETHING WRONG IN ROAD TARGET");
            // Logger(YELLOW, "current road target %d", road_target);
            // Logger(YELLOW, "current prev x target %d", prev_x_target);
            // Logger(YELLOW, "current prev spike %d", prev_spike);
            x_target = prev_x_target;
            y_target = frame_binary.rows - 200;
        }
    }
    else if (spike_final == 2 && (!must3Lines))
    {
        // Logger(GREEN, "2 LINES");
        for (int i = 0; i < in_points.size(); i++)
        {
            // checking if prev x is in one of new sliding windows then assign new index of sliding windows become road target(middle)
            if (abs(prev_x_target - in_points[i][0]) <= 100 && abs(prev_x_target - in_points[i][2]) <= 100)
            {
                // Logger(GREEN, "2 LINES DETECTED");
                road_target = i;
                // Logger(YELLOW, "New Road Target %d", i);
                problem = false;
                break;
            }
        }

        y_target = frame_binary.rows - 200;
        x_target = (in_points[road_target][0] + in_points[road_target][2]) / 2.0;

        if (road_target == 0)
        {
            prev_middle[0] = 0;
            prev_right[0] = 1;
            prev_middle[1] = (in_points[prev_middle[0]][0] + in_points[prev_middle[0]][2]) / 2.0;
            prev_right[1] = (in_points[prev_right[0]][0] + in_points[prev_right[0]][2]) / 2.0;
        }
        else if (road_target == 1)
        {
            prev_left[0] = 0;
            prev_middle[0] = 1;
            prev_left[1] = (in_points[prev_left[0]][0] + in_points[prev_left[0]][2]) / 2.0;
            prev_middle[1] = (in_points[prev_middle[0]][0] + in_points[prev_middle[0]][2]) / 2.0;
        }

        // if above cannot determine new road target from prev x in new sliding windows, problem cannot be false
        if (problem)
        {
            // Logger(RED, "SOMETHING WRONG IN ROAD TARGET");
            // Logger(YELLOW, "current road target %d", road_target);
            // Logger(YELLOW, "current prev x target %d", prev_x_target);
            // Logger(YELLOW, "current prev spike %d", prev_spike);
            x_target = prev_x_target;
            y_target = frame_binary.rows - 200;
            // must3Lines = true;
            // canbeIntercept = false;
        }
    }
    else if (spike_final == 1 && (!must3Lines))
    {
        // Logger(GREEN, "1 LINE");
        if (abs(prev_x_target - in_points[0][0]) < 100 && abs(prev_x_target - in_points[0][2]) < 100)
        {
            // Logger(GREEN, "1 LINE DETECTED");
            road_target = 0;
            // Logger(YELLOW, "New Road Target %d", 0);
            problem = false;
        }

        y_target = frame_binary.rows - 200;
        x_target = (in_points[road_target][0] + in_points[road_target][2]) / 2.0;

        if (abs(prev_left[1] - in_points[0][0]) <= 100 && abs(prev_left[1] - in_points[0][2]) <= 100)
        {
            prev_left[0] = 0;
            prev_left[1] = (in_points[prev_left[0]][0] + in_points[prev_left[0]][2]) / 2.0;
        }
        else if (abs(prev_middle[1] - in_points[0][0]) <= 100 && abs(prev_middle[1] - in_points[0][2]) <= 100)
        {
            prev_middle[0] = 0;
            prev_middle[1] = (in_points[prev_middle[0]][0] + in_points[prev_middle[0]][2]) / 2.0;
        }
        else if (abs(prev_right[1] - in_points[0][0]) <= 100 && abs(prev_right[1] - in_points[0][2]) <= 100)
        {
            prev_right[0] = 0;
            prev_right[1] = (in_points[prev_right[0]][0] + in_points[prev_right[0]][2]) / 2.0;
        }

        if (problem)
        {
            // Logger(BLUE, "ROAD TARGET < 2");
            // Logger(GREEN, "current road target %d", road_target);
            // Logger(GREEN, "current prev x target %d", prev_x_target);
            // Logger(GREEN, "current prev spike %d", prev_spike);

            SlopeIntercept(in_points[0], line_SI[0][0], line_SI[0][1]);

            // target hardcode to left or right just by looking at its slope
            if (!isnan(line_SI[0][0]))
            {
                if (line_SI[0][0] < 0)
                {
                    x_target = 600;
                    // Logger(MAGENTA, "Slope < 0");
                    prev_left[0] = 0;
                    prev_left[1] = (in_points[prev_left[0]][0] + in_points[prev_left[0]][2]) / 2.0;
                }
                else if (line_SI[0][0] > 0)
                {
                    x_target = 200;
                    // Logger(MAGENTA, "Slope > 0");
                    prev_right[0] = 0;
                    prev_right[1] = (in_points[prev_right[0]][0] + in_points[prev_right[0]][2]) / 2.0;
                }
            }
            y_target = frame_binary.rows - 200;
            must3Lines = true;
            canbeIntercept = false;
        }
    }
    else
    {
        // Logger(RED, "OFF TRACK");
        if (spike_final == 1)
        {
            SlopeIntercept(in_points[0], line_SI[0][0], line_SI[0][1]);

            // target hardcode to left or right just by looking at its slope
            if (!isnan(line_SI[0][0]))
            {
                if (line_SI[0][0] < 0)
                {
                    x_target = 600;
                    Logger(MAGENTA, "Slope < 0");
                    prev_left[0] = 0;
                    prev_left[1] = (in_points[prev_left[0]][0] + in_points[prev_left[0]][2]) / 2.0;
                }
                else if (line_SI[0][0] > 0)
                {
                    x_target = 200;
                    Logger(MAGENTA, "Slope > 0");
                    prev_right[0] = 0;
                    prev_right[1] = (in_points[prev_right[0]][0] + in_points[prev_right[0]][2]) / 2.0;
                }
            }
        }
        else if (spike_final == 2)
        {
            for (int i = 0; i < in_points.size(); i++)
            {
                // checking if prev x is in one of new sliding windows then assign new index of sliding windows become road target(middle)
                if (abs(prev_x_target - in_points[i][0]) <= 100 && abs(prev_x_target - in_points[i][2]) <= 100)
                {
                    // Logger(GREEN, "2 LINES DETECTED");
                    road_target = i;
                    // Logger(YELLOW, "New Road Target %d", i);
                    problem = false;
                    break;
                }
            }

            if (road_target == 0)
            {
                prev_middle[0] = 0;
                prev_right[0] = 1;
                prev_middle[1] = (in_points[prev_middle[0]][0] + in_points[prev_middle[0]][2]) / 2.0;
                prev_right[1] = (in_points[prev_right[0]][0] + in_points[prev_right[0]][2]) / 2.0;
            }
            else if (road_target == 1)
            {
                prev_left[0] = 0;
                prev_middle[0] = 1;
                prev_left[1] = (in_points[prev_left[0]][0] + in_points[prev_left[0]][2]) / 2.0;
                prev_middle[1] = (in_points[prev_middle[0]][0] + in_points[prev_middle[0]][2]) / 2.0;
            }
        }
    }

    if (find3Lines && !must3Lines)
    {
        must3Lines = true;
    }

    if (spike_final > 0)
    {
        // Logger(CYAN, "Preparing publishing data");

        //==Debug line with line
        if (prev_left[0] != 6969)
        {
            cv::line(frame_dst, cv::Point(in_points[prev_left[0]][0], in_points[prev_left[0]][1]), cv::Point(in_points[prev_left[0]][2], in_points[prev_left[0]][3]), cv::Scalar(255, 0, 0), 7);
        }
        if (prev_middle[0] != 6969)
        {
            cv::line(frame_dst, cv::Point(in_points[prev_middle[0]][0], in_points[prev_middle[0]][1]), cv::Point(in_points[prev_middle[0]][2], in_points[prev_middle[0]][3]), cv::Scalar(0, 255, 0), 7);
        }
        if (prev_right[0] != 6969)
        {
            cv::line(frame_dst, cv::Point(in_points[prev_right[0]][0], in_points[prev_right[0]][1]), cv::Point(in_points[prev_right[0]][2], in_points[prev_right[0]][3]), cv::Scalar(0, 0, 255), 7);
        }

        //==Debug line with circle
        // cv::circle(frame_dst, cv::Point(prev_left[1], y_target), 5, cv::Scalar(255, 0, 0), 12);
        // cv::circle(frame_dst, cv::Point(prev_middle[1], y_target), 5, cv::Scalar(0, 255, 0), 12);
        // cv::circle(frame_dst, cv::Point(prev_right[1], y_target), 5, cv::Scalar(0, 0, 255), 12);

        if (!isnan(x_target))
        {
            prev_x_target = x_target;
            target_x = x_target;
        }

        target_y = y_target;

        cv::circle(frame_dst, cv::Point(target_x, target_y), 3, cv::Scalar(0, 0, 0), 7);

        msg_collection::RealPosition lane;
        float dist_x = 800 - target_y;
        float dist_y = target_x - 400;

        float distance = pixel_to_real(sqrt(pow(dist_x, 2) + pow(dist_y, 2)));
        float angle_diff = atan(dist_x / dist_y);

        if (dist_y < 0)
            angle_diff += DEG2RAD(180);

        if (prev_left[0] != 6969)
        {
            lane.left_lane_x_top = in_points[prev_left[0]][0];
            lane.left_lane_y_top = in_points[prev_left[0]][1];
            lane.left_lane_x_bottom = in_points[prev_left[0]][2];
            lane.left_lane_y_bottom = in_points[prev_left[0]][3];
        }

        if (prev_middle[0] != 6969)
        {
            lane.middle_lane_x_top = in_points[prev_middle[0]][0];
            lane.middle_lane_y_top = in_points[prev_middle[0]][1];
            lane.middle_lane_x_bottom = in_points[prev_middle[0]][2];
            lane.middle_lane_y_bottom = in_points[prev_middle[0]][3];
        }

        if (prev_right[0] != 6969)
        {
            lane.right_lane_x_top = in_points[prev_right[0]][0];
            lane.right_lane_y_top = in_points[prev_right[0]][1];
            lane.right_lane_x_bottom = in_points[prev_right[0]][2];
            lane.right_lane_y_bottom = in_points[prev_right[0]][3];
        }

        lane.target_x = distance * sin(angle_diff);
        lane.target_y = distance * cos(angle_diff);

        lane.can_be_intercepted = canbeIntercept;

        // Logger(GREEN, "x_pub : %f y_pub : %f", lane.target_x_left, lane.target_y_left);

        pub_target.publish(lane);
        // Logger(CYAN, "Data has been published");
    }
}

void CenterSpike(cv::Mat frame, int start, int stop, int &index)
{
    // Logger(CYAN, "Center spike equation");
    int a = 0;
    int b = 0;
    int temp = 0;
    for (int i = start; i <= stop; i++)
    {
        a += frame.at<int>(0, i) * i;
        b += frame.at<int>(0, i);

        if (frame.at<int>(0, i) > temp)
        {
            temp = frame.at<int>(0, i);
            index = i;
        }
    }
    // index = (int)(a / (float)(b));
    // Logger(GREEN, "center x : %d", index);
}

void SlopeIntercept(cv::Vec4i &lines, double &slope, double &intercept)
{
    // Logger(CYAN, "Slope intercept calculation");
    double x1 = lines[0];
    double y1 = lines[1];
    double x2 = lines[2];
    double y2 = lines[3];

    slope = (y2 - y1) / (x2 - x1);
    intercept = y1 - (slope * x1);
}