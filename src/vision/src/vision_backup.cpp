/**
 * @copyright @Danendra10 & @isabellejt & @hernanda16
 * @brief This node will calculate the line of the road
 *       and publish the point of the line
 * @license IRIS
 */

#include "vision/vision.hh"
int a, b, c;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vision");
    ros::NodeHandle NH;
    image_transport::ImageTransport IT(NH);
    ros::MultiThreadedSpinner MTS(0);

    namedWindow("thresholding", WINDOW_AUTOSIZE);
    createTrackbar("Thresh", "thresholding", &a, 200);
    createTrackbar("Min Length", "thresholding", &b, 150);
    createTrackbar("Max Gap", "thresholding", &c, 50);

    Init();
    LogParams();

    tim_30hz = NH.createTimer(ros::Duration(1.0 / 30.0), Tim30HzCllbck);
    sub_raw_frame = IT.subscribe("/catvehicle/camera_front/image_raw_front", 1, SubRawFrameCllbck);
    sub_odom = NH.subscribe("/catvehicle/odom", 1, SubOdomRaw);
    sub_lidar_data = NH.subscribe("/lidar_data", 1, SubLidarData);
    sub_cmd_vision = NH.subscribe("/cmd_vision", 1, SubCmdVision);

    pub_car_pose = NH.advertise<geometry_msgs::Point>("/car_pose", 1);
    pub_target = NH.advertise<msg_collection::RealPosition>("/real_lines", 1);
    pub_slope = NH.advertise<msg_collection::SlopeIntercept>("/vision/slope", 1);

    MTS.spin();

    return 0;
}

void SubCmdVision(const msg_collection::CmdVision::ConstPtr& msg)
{
    find3Lines = msg->find_3_lanes;
}

void SubRawFrameCllbck(const sensor_msgs::ImageConstPtr& msg)
{
    mutex_raw_frame.lock();
    raw_frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    mutex_raw_frame.unlock();

    validators |= 0b001;
}

void SubOdomRaw(const nav_msgs::Odometry::ConstPtr& msg)
{
    car_pose.x = msg->pose.pose.position.x;
    car_pose.y = msg->pose.pose.position.y;
    car_pose.th = RAD2DEG(2 * atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));

    geometry_msgs::Point car_pose_msg;
    car_pose_msg.x = car_pose.x;
    car_pose_msg.y = car_pose.y;
    car_pose_msg.z = car_pose.th;
    pub_car_pose.publish(car_pose_msg);
    // ROS_ERROR("z = %f || w = %f || th = %f", msg->pose.pose.orientation.z, msg->pose.pose.orientation.w, car_pose.th);
}

void SubLidarData(const msg_collection::Obstacles::ConstPtr& msg)
{
    obstacles.clear();
    raw_obstacles.clear();
    for (int i = 0; i < msg->x.size(); i++) {
        ObstaclesPtr obstacle(new Obstacles);
        obstacle->x = msg->x[i] + car_pose.x;
        obstacle->y = msg->y[i] + car_pose.y;
        obstacles.push_back(obstacle);

        ObstaclesPtr raw_obstacle(new Obstacles);
        raw_obstacle->x = msg->x[i];
        raw_obstacle->y = msg->y[i];
        raw_obstacle->dist = msg->dist[i];
        raw_obstacles.push_back(raw_obstacle);

        // if (i % 10 == 0)
        // printf("obs %f %f || dist %f\n", msg->x[i], raw_obstacle->x, raw_obstacle->dist);
        // printf("obs %f %f || dist %f\n", raw_obstacle->x, raw_obstacle->y, raw_obstacle->dist);
    }
}

void Tim30HzCllbck(const ros::TimerEvent& event)
{
    if (validators != 0b001)
        return;

    Mat frame_resize;
    Mat frame_gray_resize;
    Mat frame_remapped = Mat(DST_REMAPPED_HEIGHT, DST_REMAPPED_WIDTH, CV_8UC1);

    vector<Point2f> roi_corners;
    vector<Point2f> midpoints(4);
    vector<Point2f> dst_corners(4);

    roi_corners.push_back(cv::Point(2, 450));
    roi_corners.push_back(cv::Point(2, 670));
    roi_corners.push_back(cv::Point(798, 670));
    roi_corners.push_back(cv::Point(798, 450));

    midpoints[0] = (roi_corners[0] + roi_corners[1]) / 2;
    midpoints[1] = (roi_corners[1] + roi_corners[2]) / 2;
    midpoints[2] = (roi_corners[2] + roi_corners[3]) / 2;
    midpoints[3] = (roi_corners[3] + roi_corners[0]) / 2;
    dst_corners[0].x = 0;
    dst_corners[0].y = 0;
    dst_corners[1].x = (float)norm(midpoints[1] - midpoints[3]);
    dst_corners[1].y = 0;
    dst_corners[2].x = dst_corners[1].x;
    dst_corners[2].y = (float)norm(midpoints[0] - midpoints[2]);
    dst_corners[3].x = 0;
    dst_corners[3].y = dst_corners[2].y;

    Size warped_image_size = Size(cvRound(dst_corners[2].x), cvRound(dst_corners[2].y));
    Mat M = getPerspectiveTransform(roi_corners, dst_corners);
    Mat warped_image;
    warpPerspective(raw_frame, warped_image, M, warped_image_size); // do perspective transformation
    imshow("Warped Image", raw_frame);

    resize(raw_frame, frame_resize, Size(cam_params.image_width, cam_params.image_height));
    cvtColor(frame_resize, frame_gray_resize, COLOR_BGR2GRAY);

    InversePerspective(DST_REMAPPED_WIDTH, DST_REMAPPED_HEIGHT, frame_gray_resize.data, maptable, frame_remapped.data);

    /*line(raw_frame, Point(cam_params.image_width >> 1, 0), Point(cam_params.image_width >> 1, cam_params.image_height), Scalar(0, 0, 255), 1);
    line(raw_frame, Point(0, cam_params.image_height >> 1), Point(cam_params.image_width, cam_params.image_height >> 1), Scalar(0, 0, 255), 1);

    LaneDetect detect(frame_remapped);

    detect.nextFrame(frame_remapped);

    Mat obs_frame = DrawObsPoints(raw_obstacles);

    Mat final_lane = detect.getResult();

    vector<Point> lanes = detect.getLanes();

    Mat lane_points = Mat::zeros(final_lane.size(), CV_8UC3);

    vector<Point> left_lane = detect.getLeftLane();

    msg_collection::PointArray lane;
    msg_collection::RealPosition real;

    for (int i = 0; i < left_lane.size(); i++)
    {
        circle(lane_points, left_lane[i], 3, Scalar(255, 0, 0), -1);
        lane.left_lane_x.push_back(left_lane[i].x);
        lane.left_lane_y.push_back(left_lane[i].y);
        real.left_lane_x_real.push_back(PxToM(700 - left_lane[i].y) + car_pose.x);
        real.left_lane_y_real.push_back(PxToM(left_lane[i].x - 400) + car_pose.y);
    }

    vector<Point> right_lane = detect.getRightLane();

    for (int i = 0; i < right_lane.size(); i++)
    {
        circle(lane_points, right_lane[i], 3, Scalar(0, 255, 0), -1);
        lane.right_lane_x.push_back(right_lane[i].x);
        lane.right_lane_y.push_back(right_lane[i].y);
        real.right_lane_x_real.push_back(PxToM(700 - right_lane[i].y) + car_pose.x);
        real.right_lane_y_real.push_back(PxToM(right_lane[i].x - 400) + car_pose.y);
    }

    vector<Point> middle_lane = detect.calcMiddleLane();
    // printf("x %d y %d ==> y %f x %f\n", middle_lane[middle_lane.size() - 1].x, middle_lane[middle_lane.size() - 1].y, PxToM(middle_lane[middle_lane.size() - 1].x - 400), PxToM(700 - middle_lane[middle_lane.size() - 1].y));
    vector<double> x_middle_lane;
    vector<double> y_middle_lane;

    for (int i = 0; i < middle_lane.size(); i++)
    {
        circle(lane_points, middle_lane[i], 3, Scalar(255, 255, 255), -1);
        lane.middle_lane_x.push_back(middle_lane[i].x);
        lane.middle_lane_y.push_back(middle_lane[i].y);
        x_middle_lane.push_back(middle_lane[i].x);
        y_middle_lane.push_back(middle_lane[i].y);
        real.middle_lane_x_real.push_back(PxToM(700 - middle_lane[i].y) + car_pose.x);
        real.middle_lane_y_real.push_back(PxToM(middle_lane[i].x - 400) + car_pose.y);
    }

    pub_lane.publish(real);

    polynom.fit(x_middle_lane, y_middle_lane);

    for (int i = 0; i < 800; i++)
    {
        double x = i;
        double y = polynom.predict(x);
        circle(lane_points, Point(x, y), 3, Scalar(0, 0, 255), -1);
    }

    vector<double> weight = polynom.getW();

    double a = weight[0];
    double b = weight[1];
    double c = weight[2];

    putText(lane_points, "equation : " + to_string(a) + "x^2 + " + to_string(b) + "x + " + to_string(c), Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1, 8, false);


    // setMouseCallback("frame_remapped", click_event);
    // imshow("frame_remapped", frame_remapped);
    // imshow("final_lane", final_lane);
    // imshow("with obs", obs_frame);
    imshow("lane_points", lane_points);*/
    // imshow("frame", raw_frame);
    // record();

    Mat line_bgr;
    cvtColor(frame_remapped, line_bgr, COLOR_GRAY2BGR);

    Detect(line_bgr);
    // Detect(raw_frame);

    waitKey(1);
}

//========================================================================================================================

void click_event(int event, int x, int y, int flags, void* params)
{
    if (event == EVENT_LBUTTONDOWN) {
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
    for (int i = 0; i < 7000; i++) {
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

//==================>> NOT USED <<==================//

Mat ToWrappedFrame(Mat raw_frame)
{
    Point pt_a = Point(2, 450);
    Point pt_b = Point(2, 670);
    Point pt_c = Point(798, 670);
    Point pt_d = Point(798, 450);

    float width_AD = sqrt(pow(pt_a.x - pt_d.x, 2) + pow(pt_a.y - pt_d.y, 2));
    float width_BC = sqrt(pow(pt_b.x - pt_c.x, 2) + pow(pt_b.y - pt_c.y, 2));

    float height_AB = sqrt(pow(pt_a.x - pt_b.x, 2) + pow(pt_a.y - pt_b.y, 2));
    float height_CD = sqrt(pow(pt_c.x - pt_d.x, 2) + pow(pt_c.y - pt_d.y, 2));

    vector<Point2f> input_pts;
    input_pts.push_back(pt_a);
    input_pts.push_back(pt_b);
    input_pts.push_back(pt_c);
    input_pts.push_back(pt_d);

    vector<Point2f> output_pts;
    output_pts.push_back(Point2f(0, 0));
    output_pts.push_back(Point2f(0, 800));
    output_pts.push_back(Point2f(800, 800));
    output_pts.push_back(Point2f(800, 0));

    Mat mask = getPerspectiveTransform(input_pts, output_pts);

    Mat wrapped_frame;

    warpPerspective(raw_frame, wrapped_frame, mask, Size(800, 800));

    return wrapped_frame;
}

vector<Point> GetPoints(Mat wrapped_frame)
{
    Mat gray, blur, edges, mask, masked_edges;

    vector<Point> points;

    cvtColor(wrapped_frame, gray, COLOR_BGR2GRAY);

    GaussianBlur(gray, blur, Size(5, 5), 0);

    Canny(blur, edges, 50, 150);

    float height = wrapped_frame.rows;
    float width = wrapped_frame.cols;

    vector<Point> roi_vertices;
    roi_vertices.push_back(Point(0, height));
    roi_vertices.push_back(Point(0, height * 1 / 5));
    roi_vertices.push_back(Point(width, height * 1 / 5));
    roi_vertices.push_back(Point(width, height));

    mask = Mat::zeros(edges.size(), edges.type());

    fillConvexPoly(mask, roi_vertices, Scalar(255, 255, 255));

    for (int i = 0; i < edges.rows; i++) {
        for (int j = 0; j < edges.cols; j++) {
            if (edges.at<uchar>(i, j) == 255) {
                points.push_back(Point(j, i));
            }
        }
    }

    return points;
}

std::vector<cv::Vec4i> GetLeftLines(const std::vector<cv::Vec4i>& lines)
{
    std::vector<cv::Vec4i> left_lines;
    int min_x = 800; // Initialize with a large value

    for (const cv::Vec4i& line : lines) {
        cv::Point p1(line[0], line[1]);
        cv::Point p2(line[2], line[3]);

        // Check if the line is within the left region
        if (p1.x <= min_x && p2.x <= min_x && p1.x <= 300 && p2.x <= 300) {
            // Check if the line is close to the previous line
            if (left_lines.empty() || std::abs(p1.x - min_x) <= 50) {
                left_lines.push_back(line);
                min_x = std::min(p1.x, p2.x);
            }
        }
    }

    return left_lines;
}

std::vector<cv::Vec4i> GetRightLines(const std::vector<cv::Vec4i>& lines, int frameWidth)
{
    std::vector<cv::Vec4i> right_lines;
    int max_x = 0; // Initialize with a small value

    for (const cv::Vec4i& line : lines) {
        cv::Point p1(line[0], line[1]);
        cv::Point p2(line[2], line[3]);

        // Check if the line is within the right region
        if (p1.x >= max_x && p2.x >= max_x && p1.x >= frameWidth - 300 && p2.x >= frameWidth - 300) {
            // Check if the line is close to the previous line
            if (right_lines.empty() || std::abs(p1.x - max_x) <= 50) {
                right_lines.push_back(line);
                max_x = std::max(p1.x, p2.x);
            }
        }
    }

    return right_lines;
}

std::vector<cv::Vec4i> GetMiddleLines(const std::vector<cv::Vec4i>& lines, int frameWidth)
{
    std::vector<cv::Vec4i> middle_lines;
    int min_x = frameWidth / 2 - 100; // Left boundary of middle region
    int max_x = frameWidth / 2 + 100; // Right boundary of middle region

    for (const cv::Vec4i& line : lines) {
        cv::Point p1(line[0], line[1]);
        cv::Point p2(line[2], line[3]);

        // Check if the line is within the middle region
        if ((p1.x >= min_x && p1.x <= max_x) || (p2.x >= min_x && p2.x <= max_x)) {
            if (middle_lines.empty() || std::abs(p1.x - p2.x) <= 50) {
                middle_lines.push_back(line);
            }
        }
    }

    return middle_lines;
}

std::vector<cv::Vec4i> GetMiddlePoints(const std::vector<cv::Vec4i>& leftLines, const std::vector<cv::Vec4i>& middleLines)
{
    std::vector<cv::Vec4i> middlePoints;

    for (size_t i = 0; i < leftLines.size() && i < middleLines.size(); ++i) {
        cv::Point2f leftLineStart(leftLines[i][0], leftLines[i][1]);
        cv::Point2f leftLineEnd(leftLines[i][2], leftLines[i][3]);

        cv::Point2f middleLineStart(middleLines[i][0], middleLines[i][1]);
        cv::Point2f middleLineEnd(middleLines[i][2], middleLines[i][3]);

        cv::Point2f middlePt = 0.5f * (leftLineStart + middleLineStart);

        middlePoints.push_back(cv::Vec4i(middlePt.x, middlePt.y));
    }

    return middlePoints;
}

cv::Vec4i ExtrapolateLine(const cv::Vec4i& line, int minY, int maxY)
{
    double slope = static_cast<double>(line[3] - line[1]) / static_cast<double>(line[2] - line[0]);
    int startX = line[0] + static_cast<int>((minY - line[1]) / slope);
    int endX = line[0] + static_cast<int>((maxY - line[1]) / slope);
    return cv::Vec4i(startX, minY, endX, maxY);
}

std::vector<cv::Point> GetLeftPoints(const std::vector<cv::Point>& points)
{
    std::vector<cv::Point> left_points;
    int min_x = 800; // Initialize with a large value

    for (const cv::Point& point : points) {
        if (point.y < 100)
            continue;
        // Check if the point is within the left region
        if (point.x <= min_x && point.x <= 300) {
            // Check if the point is close to the previous point
            if (left_points.empty() || std::abs(point.x - min_x) <= 50) {
                left_points.push_back(point);
                min_x = point.x;
            }
        }
    }

    return left_points;
}

std::vector<cv::Point> GetRightPoints(const std::vector<cv::Point>& points, int frameWidth)
{
    std::vector<cv::Point> right_points;
    int max_x = 0; // Initialize with a small value

    for (const cv::Point& point : points) {
        if (point.y < 100)
            continue;
        // Check if the point is within the right region
        if (point.x >= max_x && point.x >= frameWidth - 300) {
            // Check if the point is close to the previous point
            if (right_points.empty() || std::abs(point.x - max_x) <= 50) {
                right_points.push_back(point);
                max_x = point.x;
            }
        }
    }

    return right_points;
}

std::vector<cv::Point> GetMiddlePoints(const std::vector<cv::Point>& points, int frameWidth)
{
    std::vector<cv::Point> middle_points;
    int min_x = frameWidth / 2 - 100; // Left boundary of middle region
    int max_x = frameWidth / 2 + 100; // Right boundary of middle region

    for (const cv::Point& point : points) {
        if (point.y < 100)
            continue;
        // check if the point exist in the left and right region
        for (const cv::Point& left_point : left_points) {
            if (point.x == left_point.x && point.y == left_point.y) {
                continue;
            }
        }
        for (const cv::Point& right_point : right_points) {
            if (point.x == right_point.x && point.y == right_point.y) {
                continue;
            }
        }
        if (point.x >= min_x && point.x <= max_x) {

            if (middle_points.empty() || std::abs(point.x - point.x) <= 50) {
                middle_points.push_back(point);
            }
        }
    }

    return middle_points;
}

std::vector<cv::Point> GetMiddleOfLeftRoad(const std::vector<cv::Point>& leftPoints, const std::vector<cv::Point>& middlePoints)
{
    std::vector<cv::Point> middleOfLeftRoad;

    for (size_t i = 0; i < leftPoints.size() && i < middlePoints.size(); ++i) {
        cv::Point2f leftPoint(leftPoints[i].x, leftPoints[i].y);
        cv::Point2f middlePoint(middlePoints[i].x, middlePoints[i].y);

        cv::Point2f middlePt = 0.5f * (leftPoint + middlePoint);

        middleOfLeftRoad.push_back(middlePt);
    }

    return middleOfLeftRoad;
}

std::vector<cv::Point> GetMiddleOfRightRoad(const std::vector<cv::Point>& rightPoints, const std::vector<cv::Point>& middlePoints)
{
    std::vector<cv::Point> middleOfRightRoad;

    for (size_t i = 0; i < rightPoints.size() && i < middlePoints.size(); ++i) {
        cv::Point2f rightPoint(rightPoints[i].x, rightPoints[i].y);
        cv::Point2f middlePoint(middlePoints[i].x, middlePoints[i].y);

        cv::Point2f middlePt = 0.5f * (rightPoint + middlePoint);

        middleOfRightRoad.push_back(middlePt);
    }

    return middleOfRightRoad;
}

Mat DrawObsPoints(const vector<ObstaclesPtr>& points)
{
    Mat frame = Mat::zeros(800, 800, CV_8UC3);
    // draw car in the middle
    float car_x = 400;
    float car_y = 700;

    cv::circle(frame, cv::Point(car_x, car_y), 30, cv::Scalar(0, 255, 0), 2);

    for (int i = 0; i < points.size(); i++) {
        float obs_y = (700 - points[i]->x * 30);
        float obs_x = (points[i]->y * 60 + 400);

        // printf("obs frame x %.2f y %.2f\n", obs_x, obs_y);
        // printf("points obs x %.2f y %.2f\n", points[i]->x, points[i]->y);
        cv::circle(frame, cv::Point(obs_x, obs_y), 5, cv::Scalar(0, 0, 255), -1);
    }

    for (int i = 0; i < middle_points.size(); i++) {
        cv::circle(frame, cv::Point(middle_points[i].x, middle_points[i].y), 5, cv::Scalar(255, 0, 0), -1);
    }

    // robot safe areas
    // float safe_angle = 10;
    // cv::line(frame, cv::Point(300, 400), cv::Point(400, 700), cv::Scalar(255, 0, 0), 2);
    // cv::line(frame, cv::Point(500, 400), cv::Point(400, 700), cv::Scalar(255, 0, 0), 2);

    return frame;
}

void Detect(cv::Mat frame)
{
    cv::Mat frame_gray, frame_canny, frame_thresh, frame_far;
    cv::Mat result = frame.clone();
    std::vector<cv::Vec4i> line_hough;

    cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(frame_gray, frame_gray, cv::Size(5, 5), 0);

    //==Option Threshold
    cv::threshold(frame_gray, frame_thresh, 55, 255, cv::THRESH_BINARY);
    erode(frame_thresh, frame_thresh, Mat(), Point(-1, -1), 1);
    dilate(frame_thresh, frame_thresh, Mat(), Point(-1, -1), 1);
    frame_far = frame_thresh.clone();
    ROI(frame_thresh, frame_far);

    //==Option Edge
    // cv::Canny(frame_far, frame_canny, 25, 50);
    // ROI(frame_canny, frame_far);

    //==Method
    // Hough(frame_canny, line_hough);
    if (!frame_thresh.empty()) {
        BinaryStacking(frame_thresh, result);
    }
    // if (!frame_far.empty()) {
    //     BinaryStacking(frame_far, result);
    // }

    // SlidingWindows(result, line_hough);
    // Display(result, line_hough, 0, 255, 0, 0.2);
    // Average(result, line_hough);
    // Display(result, line_hough, 255, 255, 255, 0.5);
    // setMouseCallback("result", click_event);

    cv::Mat frame_canny_resized;
    cv::Mat frame_thresh_resized;
    cv::Mat frame_far_resized;
    cv::Mat result_resized;

    if (!frame_canny.empty()) {
        cv::resize(frame_canny, frame_canny_resized, cv::Size(400, 400));
        cv::imshow("edge", frame_canny_resized);
    }
    if (!frame_thresh.empty()) {
        cv::resize(frame_thresh, frame_thresh_resized, cv::Size(400, 400));
        // cv::imshow("thresh", frame_thresh_resized);
        cv::cvtColor(frame_thresh, frame_thresh, cv::COLOR_GRAY2BGR);
        cv::addWeighted(frame_thresh, 0.3, result, 1.0, 0.0, result);
    }
    if (!frame_far.empty()) {
        cv::resize(frame_far, frame_far_resized, cv::Size(400, 400));
        cv::imshow("far", frame_far_resized);
    }
    if (!result.empty()) {
        cv::resize(result, result_resized, cv::Size(400, 400));
        cv::imshow("result", result_resized);
    }
}

void ROI(cv::Mat& frame, cv::Mat& frame_faraway)
{
    cv::Mat frame_mask(frame.rows, frame.cols, CV_8UC1, cv::Scalar(0));
    cv::Mat frame_mask_far(frame.rows, frame.cols, CV_8UC1, cv::Scalar(0));
    std::vector<cv::Point> ROI;

    ROI.push_back(cv::Point(100, 700)); // bottom left
    ROI.push_back(cv::Point(700, 700)); // bottom right
    ROI.push_back(cv::Point(700, 500)); // top right
    ROI.push_back(cv::Point(100, 500)); // top left

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

    // normal frame
    //  ROI.push_back(cv::Point(0, frame.rows / 2 + 40)); //top left
    //  ROI.push_back(cv::Point(frame.cols, frame.rows / 2 + 40)); //top right
    //  ROI.push_back(cv::Point(frame.cols, frame.rows - 122)); //bottom right
    //  ROI.push_back(cv::Point(0, frame.rows - 122)); //bottom left

    // std::vector<cv::Point> Ignore;

    // if (road_part == LeftLane) {
    //     Ignore.push_back(cv::Point(frame.cols, frame.rows - 220)); //center right
    //     Ignore.push_back(cv::Point(frame.cols / 2 + 100, frame.rows / 2 + 70)); //top right
    //     Ignore.push_back(cv::Point(frame.cols / 2, frame.rows / 2 + 70)); //top left
    //     Ignore.push_back(cv::Point(frame.cols / 2 + 100, frame.rows - 122)); //bottom left
    //     Ignore.push_back(cv::Point(frame.cols, frame.rows - 122)); //bottom right
    // } else if (road_part == RightLane) {
    //     Ignore.push_back(cv::Point(0, frame.rows - 220)); //center left
    //     Ignore.push_back(cv::Point(frame.cols / 2 - 100, frame.rows / 2 + 70)); //top left
    //     Ignore.push_back(cv::Point(frame.cols / 2, frame.rows / 2 + 70)); //top right
    //     Ignore.push_back(cv::Point(frame.cols / 2 - 100, frame.rows - 122)); //bottom right
    //     Ignore.push_back(cv::Point(0, frame.rows - 122)); //bottom left
    // }

    fillConvexPoly(frame_mask, ROI, cv::Scalar(255));
    fillConvexPoly(frame_mask_far, ROI_left, cv::Scalar(255));
    fillConvexPoly(frame_mask_far, ROI_right, cv::Scalar(255));
    fillConvexPoly(frame_mask_far, ROI_center, cv::Scalar(255));
    // fillConvexPoly(frame_mask, Ignore, cv::Scalar(0));
    // cv::rectangle(frame_mask, cv::Point(245, 795), cv::Point(555, 636), cv::Scalar(0), CV_FILLED);
    cv::bitwise_and(frame, frame_mask, frame);
    cv::bitwise_and(frame_faraway, frame_mask_far, frame_faraway);

    // cv::imshow("mask", frame_mask);
    // cv::imshow("mask far", frame_mask_far);
}

void Hough(cv::Mat frame, std::vector<cv::Vec4i>& line)
{
    cv::HoughLinesP(frame, line, 2, CV_PI / 180, 94, 36, 14); // 100,115,15
}

void Display(cv::Mat& frame, std::vector<cv::Vec4i> lines, int b_, int g_, int r_, float intensity)
{
    cv::Mat draw(frame.rows, frame.cols, CV_8UC3, cv::Scalar(0));
    if (lines.size() > 0)
        for (size_t i = 0; i < lines.size(); i++) {
            cv::Vec4i line = lines[i];
            cv::line(draw, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(b_, g_, r_), 10);
        }

    cv::addWeighted(draw, intensity, frame, 1.0, 0.0, frame);

    // cv::imshow("line", draw);
}

void Average(cv::Mat frame, std::vector<cv::Vec4i>& lines)
{
    std::vector<cv::Vec2f> left_fit;
    std::vector<cv::Vec2f> right_fit;
    int temp = 0;
    double slope;

    // for (size_t i = 0; i < lines.size(); i++)
    // {
    //     double x1 = lines[i][0];
    //     double y1 = lines[i][1];
    //     double x2 = lines[i][2];
    //     double y2 = lines[i][3];

    //     double slope = (y2-y1)/(x2-x1);
    //     double intercept = y1-(slope*x1);

    //     // Print the coefficients of the fitted polynomial
    //     // std::cout << "Slope: " << slope << std::endl;
    //     // std::cout << "Intercept: " << coeffs[1] << std::endl;

    //     if(slope < 0 && lines[0][0] >= frame.cols/4.0)
    //     {
    //         lines.erase(lines.begin()+(i-temp));
    //         temp++;
    //     }
    //     else if(slope > 0 && lines[0][0] <= 3*frame.cols/4.0)
    //     {
    //         lines.erase(lines.begin()+(i-temp));
    //         temp++;
    //     }
    // }

    for (size_t i = 0; i < lines.size(); i++) {
        double x1 = lines[i][0];
        double y1 = lines[i][1];
        double x2 = lines[i][2];
        double y2 = lines[i][3];

        slope = (y2 - y1) / (x2 - x1);
        double intercept = y1 - (slope * x1);

        // Print the coefficients of the fitted polynomial
        // std::cout << "Slope: " << slope << std::endl;
        // std::cout << "Intercept: " << coeffs[1] << std::endl;

        // std::cout << x1 << " " << x2 <<std::endl;
        if (slope < 0 && x1 < 350 && x2 < 350) {
            left_fit.push_back(cv::Vec2f(slope, intercept));
        } else if (slope > 0 && x1 > 350 && x2 > 350) {
            right_fit.push_back(cv::Vec2f(slope, intercept));
        }
    }

    cv::Vec2f left_fit_avg = VectorAvg(left_fit);
    cv::Vec2f right_fit_avg = VectorAvg(right_fit);
    cv::Vec2f mid_fit_avg, mid_fit_avg_left, mid_fit_avg_right;
    cv::Vec2f target_fit_avg;

    // std::cout<<"Left : "<<left_fit_avg<<std::endl;
    // std::cout<<"Right : "<<right_fit_avg<<std::endl;

    std::vector<cv::Vec4i> line_left = MakePoints(frame, left_fit_avg);
    std::vector<cv::Vec4i> line_right = MakePoints(frame, right_fit_avg);
    std::vector<cv::Vec4i> line_mid, line_mid_left, line_mid_right, line_target;

    // std::cout<<line_left[0]<<" "<<line_right[0]<<std::endl;
    if (!std::isnan(left_fit_avg[0]) && !std::isnan(right_fit_avg[0])) {

        prev_state = Normal;
        isWait = false;
        decision = LeftLane;

        int x1 = (line_left[0][0] + line_right[0][0]) / 2.0;
        int y1 = (line_left[0][1] + line_right[0][1]) / 2.0;
        int x2 = (line_left[0][2] + line_right[0][2]) / 2.0;
        int y2 = (line_left[0][3] + line_right[0][3]) / 2.0;
        line_mid.push_back(cv::Vec4i(x1, y1, x2, y2));

        int x1_left = (line_mid[0][0] + line_left[0][0]) / 2.0;
        int y1_left = (line_mid[0][1] + line_left[0][1]) / 2.0;
        int x2_left = (line_mid[0][2] + line_left[0][2]) / 2.0;
        int y2_left = (line_mid[0][3] + line_left[0][3]) / 2.0;
        line_mid_left.push_back(cv::Vec4i(x1_left, y1_left, x2_left, y2_left));

        int x1_right = (line_mid[0][0] + line_right[0][0]) / 2.0;
        int y1_right = (line_mid[0][1] + line_right[0][1]) / 2.0;
        int x2_right = (line_mid[0][2] + line_right[0][2]) / 2.0;
        int y2_right = (line_mid[0][3] + line_right[0][3]) / 2.0;
        line_mid_right.push_back(cv::Vec4i(x1_right, y1_right, x2_right, y2_right));

        x_target_left = x2_left;
        y_target_left = y2_left;
        x_target_right = x2_right;
        y_target_right = y2_right;

        cv::circle(frame, cv::Point(x_target_left, y_target_left), 5, cv::Scalar(255, 255, 0), 10);
        cv::circle(frame, cv::Point(x_target_right, y_target_right), 5, cv::Scalar(0, 255, 255), 10);
        // std::cout<<x_target<<"  "<<y_target<<std::endl;
        msg_collection::RealPosition lane;
        float dist_x_left = 800 - y_target_left;
        float dist_y_left = x_target_left - 400;
        float dist_x_right = 800 - y_target_right;
        float dist_y_right = x_target_right - 400;

        float distance_left = pixel_to_real(sqrt(pow(dist_x_left, 2) + pow(dist_y_left, 2)));
        float distance_right = pixel_to_real(sqrt(pow(dist_x_right, 2) + pow(dist_y_right, 2)));
        float angle_diff_left = atan(dist_x_left / dist_y_left);
        float angle_diff_right = atan(dist_x_right / dist_y_right);

        if (dist_y_left < 0)
            angle_diff_left += DEG2RAD(180);
        if (dist_y_right < 0)
            angle_diff_right += DEG2RAD(180);

        // printf("angle %f dist %f\n", RAD2DEG(angle_diff), distance);

        lane.target_x_left = distance_left * sin(angle_diff_left);
        lane.target_y_left = distance_left * cos(angle_diff_left);
        lane.target_x_right = distance_right * sin(angle_diff_right);
        lane.target_y_right = distance_right * cos(angle_diff_right);

        // printf("bef %f %f || nnnn %f %f ||diff %f\n", dist_x_right, dist_y_right, lane.target_x_right, lane.target_y_right, RAD2DEG(angle_diff_right));

        pub_target.publish(lane);
    } else if (std::isnan(left_fit_avg[0]) && !std::isnan(right_fit_avg[0]) && !isWait) {
        ROS_WARN("KIRI HILANG");

        prev_state = LeftLost;
        decision = RightLane;

        if (right_fit_avg[0] <= 1) {
            mid_fit_avg[0] = sqrt(right_fit_avg[0]);
            mid_fit_avg[1] = right_fit_avg[1] + 80;
            line_mid = MakePoints(frame, mid_fit_avg);
        } else {
            mid_fit_avg[0] = pow(right_fit_avg[0], 2);
            mid_fit_avg[1] = right_fit_avg[1] + 80;
            line_mid = MakePoints(frame, mid_fit_avg);
        }

        if (mid_fit_avg[0] <= 1) {
            target_fit_avg[0] = pow(right_fit_avg[0], (1.0 / 3.0));
            target_fit_avg[1] = mid_fit_avg[1] + 80;
            line_mid_left = MakePoints(frame, target_fit_avg);
        } else {
            target_fit_avg[0] = pow(left_fit_avg[0], 3);
            target_fit_avg[1] = mid_fit_avg[1] + 80;
            line_mid_left = MakePoints(frame, target_fit_avg);
        }

        int mid_x1 = (line_mid[0][0] + line_right[0][0]) / 2.0;
        int mid_y1 = (line_mid[0][1] + line_right[0][1]) / 2.0;
        int mid_x2 = (line_mid[0][2] + line_right[0][2]) / 2.0;
        int mid_y2 = (line_mid[0][3] + line_right[0][3]) / 2.0;
        line_mid_right.push_back(cv::Vec4i(mid_x1, mid_y1, mid_x2, mid_y2));

        x_target_left = line_mid_left[0][2];
        y_target_left = line_mid_left[0][3];
        x_target_right = line_mid_right[0][2];
        y_target_right = line_mid_right[0][3];

        // if(x_target_buf == 0 && y_target_buf == 0)
        // {
        //     x_target_buf = x_target;
        //     y_target_buf = y_target;
        // }

        // if(abs(x_target-x_target_buf)>500)
        // {
        //     x_target_buf = x_target;
        //     y_target_buf = y_target;
        // }

        // printf("Target %d %d || buff target %d %d\n", x_target, y_target, x_target_buf, y_target_buf);

        // if(abs(x_target-x_target_buf)>200)
        // {
        //     x_target=x_target_buf;
        //     y_target=y_target_buf;
        // }

        // x_target_buf = x_target;
        // y_target_buf = y_target;

        cv::circle(frame, cv::Point(x_target_left, y_target_left), 5, cv::Scalar(255, 255, 0), 10);
        cv::circle(frame, cv::Point(x_target_right, y_target_right), 5, cv::Scalar(0, 255, 255), 10);

        // std::cout<<x_target<<"  "<<y_target<<std::endl;
        msg_collection::RealPosition lane;
        float dist_x_left = 800 - y_target_left;
        float dist_y_left = x_target_left - 400;
        float dist_x_right = 800 - y_target_right;
        float dist_y_right = x_target_right - 400;

        float distance_left = pixel_to_real(sqrt(pow(dist_x_left, 2) + pow(dist_y_left, 2)));
        float distance_right = pixel_to_real(sqrt(pow(dist_x_right, 2) + pow(dist_y_right, 2)));
        float angle_diff_left = atan(dist_x_left / dist_y_left);
        float angle_diff_right = atan(dist_x_right / dist_y_right);

        if (dist_y_left < 0)
            angle_diff_left += DEG2RAD(180);
        if (dist_y_right < 0)
            angle_diff_right += DEG2RAD(180);

        // printf("angle %f dist %f\n", RAD2DEG(angle_diff), distance);

        lane.target_x_left = distance_left * sin(angle_diff_left);
        lane.target_y_left = distance_left * cos(angle_diff_left);
        lane.target_x_right = distance_right * sin(angle_diff_right);
        lane.target_y_right = distance_right * cos(angle_diff_right);

        // printf("bef %f %f || nnnn %f %f\n", dist_x, dist_y, lane.target_x, lane.target_y);

        pub_target.publish(lane);
    } else if (std::isnan(right_fit_avg[0]) && !std::isnan(left_fit_avg[0]) && !isWait) {
        ROS_WARN("KANAN HILANG");

        prev_state = RightLost;
        decision = LeftLane;

        if (left_fit_avg[0] >= -1) {
            mid_fit_avg[0] = -sqrt(abs(left_fit_avg[0]));
            mid_fit_avg[1] = left_fit_avg[1] + 160;
            line_mid = MakePoints(frame, mid_fit_avg);
        } else {
            mid_fit_avg[0] = pow(left_fit_avg[0], 2);
            mid_fit_avg[1] = left_fit_avg[1] + 160;
            line_mid = MakePoints(frame, mid_fit_avg);
        }

        if (mid_fit_avg[0] >= -1) {
            target_fit_avg[0] = -pow(abs(left_fit_avg[0]), (1.0 / 3.0));
            target_fit_avg[1] = mid_fit_avg[1] + 160;
            line_mid_right = MakePoints(frame, target_fit_avg);
        } else {
            target_fit_avg[0] = pow(left_fit_avg[0], 3);
            target_fit_avg[1] = mid_fit_avg[1] + 160;
            line_mid_right = MakePoints(frame, target_fit_avg);
        }

        int mid_x1 = (line_mid[0][0] + line_left[0][0]) / 2.0;
        int mid_y1 = (line_mid[0][1] + line_left[0][1]) / 2.0;
        int mid_x2 = (line_mid[0][2] + line_left[0][2]) / 2.0;
        int mid_y2 = (line_mid[0][3] + line_left[0][3]) / 2.0;
        line_mid_left.push_back(cv::Vec4i(mid_x1, mid_y1, mid_x2, mid_y2));

        // int mid_x1 = (-abs(line_mid[0][0] / 2.0) + line_mid[0][0] + line_left[0][0]) / 2.0;
        // int mid_y1 = (line_mid[0][1] + line_left[0][1]) / 2.0;
        // int mid_x2 = (-abs(line_mid[0][2] / 2.0) + line_mid[0][2] + line_left[0][2]) / 2.0;
        // int mid_y2 = (line_mid[0][3] + line_left[0][3]) / 2.0;

        // line_target.push_back(cv::Vec4i(mid_x1, mid_y1, mid_x2, mid_y2));

        x_target_left = line_mid_left[0][2];
        y_target_left = line_mid_left[0][3];
        x_target_right = line_mid_right[0][2];
        y_target_right = line_mid_right[0][3];

        cv::circle(frame, cv::Point(x_target_left, y_target_left), 5, cv::Scalar(255, 255, 0), 10);
        cv::circle(frame, cv::Point(x_target_right, y_target_right), 5, cv::Scalar(0, 255, 255), 10);

        // std::cout<<x_target<<"  "<<y_target<<std::endl;
        msg_collection::RealPosition lane;
        float dist_x_left = 800 - y_target_left;
        float dist_y_left = x_target_left - 400;
        float dist_x_right = 800 - y_target_right;
        float dist_y_right = x_target_right - 400;

        float distance_left = pixel_to_real(sqrt(pow(dist_x_left, 2) + pow(dist_y_left, 2)));
        float distance_right = pixel_to_real(sqrt(pow(dist_x_right, 2) + pow(dist_y_right, 2)));
        float angle_diff_left = atan(dist_x_left / dist_y_left);
        float angle_diff_right = atan(dist_x_right / dist_y_right);

        if (dist_y_left < 0)
            angle_diff_left += DEG2RAD(180);
        if (dist_y_right < 0)
            angle_diff_right += DEG2RAD(180);

        // printf("angle %f dist %f\n", RAD2DEG(angle_diff), distance);

        lane.target_x_left = distance_left * sin(angle_diff_left);
        lane.target_y_left = distance_left * cos(angle_diff_left);
        lane.target_x_right = distance_right * sin(angle_diff_right);
        lane.target_y_right = distance_right * cos(angle_diff_right);

        // printf("bef %f %f || nnnn %f %f\n", dist_x, dist_y, lane.target_x, lane.target_y);

        pub_target.publish(lane);

        // std::cout<<x1<<" "<<x2<<std::endl;
    } else {
        ROS_ERROR("WATEFAK");

        if (prev_state == Normal) {
            // printf("normal\n");
            x_target_left = frame.cols / 2;
            y_target_left = frame.rows;
        } else if (prev_state == LeftLost) {
            // printf("EXTREME TO RIGHT\n");
            isWait = true;
            x_target_left = 0;
            y_target_left = 3 * frame.rows / 5.0;
        } else if (prev_state == RightLost) {
            // printf("EXTREME to LEFT\n");
            isWait = true;
            x_target_left = frame.cols;
            y_target_left = 3 * frame.rows / 5.0;
        }

        cv::circle(frame, cv::Point(x_target_left, y_target_left), 5, cv::Scalar(0, 0, 255), 10);

        msg_collection::RealPosition lane;
        float dist_x_left = 800 - y_target_left;
        float dist_y_left = x_target_left - 400;
        float dist_x_right = 800 - y_target_right;
        float dist_y_right = x_target_right - 400;

        float distance_left = pixel_to_real(sqrt(pow(dist_x_left, 2) + pow(dist_y_left, 2)));
        float distance_right = pixel_to_real(sqrt(pow(dist_x_right, 2) + pow(dist_y_right, 2)));
        float angle_diff_left = atan(dist_x_left / dist_y_left);
        float angle_diff_right = atan(dist_x_right / dist_y_right);

        if (dist_y_left < 0)
            angle_diff_left += DEG2RAD(180);
        if (dist_y_right < 0)
            angle_diff_right += DEG2RAD(180);

        // printf("angle %f dist %f\n", RAD2DEG(angle_diff), distance);

        lane.target_x_left = distance_left * sin(angle_diff_left);
        lane.target_y_left = distance_left * cos(angle_diff_left);
        lane.target_x_right = distance_right * sin(angle_diff_right);
        lane.target_y_right = distance_right * cos(angle_diff_right);

        // printf("bef %f %f || nnnn %f %f\n", dist_x_left, dist_y_left, lane.target_x_left, lane.target_y_left);

        pub_target.publish(lane);
    }

    // std::cout << isWait << std::endl;
    // printf("SLope left fit %f right %f\n", left_fit_avg[0])

    // std_msgs::Float32 msg_slope;
    // msg_slope.data = slope;
    // pub_slope.publish(msg_slope);

    // std::cout << left_fit_avg << " " << mid_fit_avg << " " << right_fit_avg << std::endl;

    Display(frame, line_left, 255, 0, 0, 1);
    Display(frame, line_mid, 0, 255, 0, 1);
    Display(frame, line_right, 0, 0, 255, 1);
    Display(frame, line_mid_left, 255, 255, 0, 1);
    Display(frame, line_mid_right, 0, 255, 255, 1);
}

cv::Vec2f VectorAvg(std::vector<cv::Vec2f> in_vec)
{
    float avg_x = 0;
    float avg_y = 0;
    for (int i = 0; i < in_vec.size(); i++) {
        avg_x += in_vec[i][0];
        avg_y += in_vec[i][1];
    }
    avg_x /= in_vec.size();
    avg_y /= in_vec.size();
    return cv::Vec2f(avg_x, avg_y);
}

std::vector<cv::Vec4i> MakePoints(cv::Mat frame, cv::Vec2f lineSI)
{
    float slope = lineSI[0];
    float intercept = lineSI[1];
    int y1 = frame.rows - 100;
    int y2 = (int)(3 * frame.rows / 5.0);
    int x1 = (int)((y1 - intercept) / slope);
    int x2 = (int)((y2 - intercept) / slope);

    return std::vector<cv::Vec4i> { cv::Vec4i(x1, y1, x2, y2) };
}

std::vector<cv::Vec4i> SlidingWindows(cv::Mat& frame, std::vector<int> x_final, std::vector<int> nonzero_x, std::vector<int> nonzero_y)
{
    std::vector<cv::Rect> windows;
    const int num_windows = 9;
    const int margin = 50;
    const int min_px = 100;
    int window_height = (int)((700 - 500) / (float)num_windows); // from ROI
    std::vector<int> current_x;
    std::vector<std::vector<int>> in_window(500);
    cv::Vec2i win_y;

    for (int i = 0; i < x_final.size(); i++) {
        current_x.push_back(x_final[i]);
        // Logger(RED, "current_x : %d", current_x[i]);
    }

    std::vector<cv::Vec2i> win_x(current_x.size());
    for (int i = 0; i < num_windows; i++) {
        win_y[0] = 700 - (i + 1) * window_height; // from ROI
        win_y[1] = 700 - i * window_height; // from ROI
        for (int j = 0; j < current_x.size(); j++) {
            win_x[j][0] = current_x[j] - margin;
            win_x[j][1] = current_x[j] + margin;
            // std::cout << win_x[j] << std::endl;
            cv::rectangle(frame, cv::Rect(win_x[j][0], win_y[0], win_x[j][1] - win_x[j][0], win_y[1] - win_y[0]), cv::Scalar(255, 0, 0), 2);

            for (int k = 0; k < nonzero_x.size(); k++) {
                if (nonzero_x[k] >= win_x[j][0] && nonzero_x[k] < win_x[j][1] && nonzero_y[k] >= win_y[0] && nonzero_y[k] < win_y[1]) {
                    in_window[j].push_back(k);
                }
            }

            if (in_window[j].size() > 100) {
                int min_y = 800;
                int max_y = 0;
                int top_x;
                int bottom_x;
                int temp = win_y[1];
                int sum = 0;
                // for (int k = 0; k < in_window[j].size(); k++) {
                //     sum += nonzero_x[in_window[j][k]];
                // }
                // current_x[j] = (int)(sum / (float)(in_window[j].size()));
                // Logger(GREEN, "current_x[%d] : %d", j, current_x[j]);

                for (int k = 0; k < in_window[j].size(); k++) {
                    if (nonzero_y[in_window[j][k]] > max_y) {
                        bottom_x = nonzero_x[in_window[j][k]];
                    }
                    if (nonzero_y[in_window[j][k]] < min_y) {
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

    for (int i = 0; i < current_x.size(); i++) {
        int temp_y_max = 0;
        int temp_y_min = 800;
        for (int j = 0; j < in_window[i].size(); j++) {
            // in_points[i][0] = nonzero_x[in_window[i][j]];
            // in_points[i][1] = nonzero_y[in_window[i][j]];
            // cv::circle(frame, cv::Point(in_points[i][0], in_points[i][1]), 1, cv::Scalar((i + 1) * 80, 0, 0), 10);
            if (nonzero_y[in_window[i][j]] < temp_y_min) {
                temp_y_min = nonzero_y[in_window[i][j]];
                in_points[i][0] = nonzero_x[in_window[i][j]];
            }
            if (nonzero_y[in_window[i][j]] > temp_y_max) {
                temp_y_max = nonzero_y[in_window[i][j]];
                in_points[i][2] = nonzero_x[in_window[i][j]];
            }
        }
        in_points[i][1] = temp_y_min;
        in_points[i][3] = temp_y_max;
    }

    return in_points;
}

/**
 * This function is used to ....
 */
void BinaryStacking(cv::Mat frame, cv::Mat& frame_dst)
{
    std::vector<cv::Vec2i> nonzero;
    cv::findNonZero(frame, nonzero);

    std::vector<int> nonzero_y(nonzero.size());
    std::vector<int> nonzero_x(nonzero.size());

    /**
     * This for loop is used to ....
     */
    for (int i = 0; i < nonzero.size(); i++) {
        nonzero_x[i] = nonzero[i][0];
        nonzero_y[i] = nonzero[i][1];
    }

    cv::Mat verticalSum;
    cv::Mat binaryMask = (frame > 0) / 255;
    cv::reduce(binaryMask, verticalSum, 0, cv::REDUCE_SUM, CV_32S);

    // std::cout << verticalSum << std::endl;

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

    for (size_t i = 0; i < verticalSum.cols; i++) {
        prev_isZero = isZero;
        if (verticalSum.at<int>(0, i) > 0 && prev_isZero) {
            isZero = false;
            start.push_back(i);
            spike++;
        } else if (verticalSum.at<int>(0, i) == 0 && !prev_isZero) {
            isZero = true;
            stop.push_back(i);
        }
    }

    for (int i = 0; i < spike; i++) {
        // Logger(YELLOW, "start : %d || stop : %d", start[i], stop[i]);
    }

    // Logger(RED, "spike : %d", spike);

    center_x_base.resize(spike);

    for (int i = 0; i < spike; i++) {
        CenterSpike(verticalSum, start[i], stop[i], center_x_base[i]);
        // Logger(BLUE, "center x[%d] : %d", i, center_x_base[i]);
        if (center_x_base[i] - center_x_base[i - 1] < 100 && i != 0) {
            center_x_final.pop_back();
            center_x_final.push_back((center_x_base[i] + center_x_base[i - 1]) / 2.0);
            counter++;
        } else {
            center_x_final.push_back(center_x_base[i]);
        }

        //==Debug Sliding Windows Coverage with Noise
        cv::circle(frame_dst, cv::Point(center_x_base[i], 720), 3, cv::Scalar(255, 0, 0), 10);
        cv::circle(frame_dst, cv::Point(center_x_base[i], 460), 3, cv::Scalar(255, 0, 0), 10);
    }

    spike_final = spike - counter;

    //==Debug Sliding Windows Coverage Final
    for (int i = 0; i < spike_final; i++) {
        cv::circle(frame_dst, cv::Point(center_x_final[i], 700), 3, cv::Scalar(i * 50, 255, 0), 10);
        cv::circle(frame_dst, cv::Point(center_x_final[i], 440), 3, cv::Scalar(i * 50, 255, 0), 10);
    }

    std::vector<cv::Vec4i> in_points = SlidingWindows(frame_dst, center_x_final, nonzero_x, nonzero_y);
    std::vector<std::vector<cv::Vec4i>> lane(in_points.size());

    // just for drawing
    for (int i = 0; i < in_points.size(); i++) {
        lane[i].push_back(in_points[i]);
        // Display(frame_dst, lane[i], i * 100, 1 * 100, 255, 0.5);
        // SlopeIntercept(in_points[i], line_SI[i][0], line_SI[i][1]); //0 -> left, 1-> middle, 2 -> right
        // cv::circle(frame_dst, cv::Point((int)((frame.rows - 220 - line_SI[i][1]) / line_SI[i][0]), frame.rows - 220), 3, cv::Scalar(i * 100, i * 100, 255), 7);
    }

    for (int i = 0; i < 3; i++) {
        line_SI[i][0] = 0;
        line_SI[i][1] = 0;
    }

    prev_left[0] = 6969;
    prev_middle[0] = 6969;
    prev_right[0] = 6969;

    // problem is used to debug if road target has problem not being able to identify new road target from prev x in new sliding windows
    problem = true;

    if (spike_final == 3) {
        Logger(GREEN, "3 LINES DETECTED");
        road_target = 1;
        must3Lines = false;
        canbeIntercept = true;
        problem = false;

        for (int i = 0; i < in_points.size(); i++) {
            SlopeIntercept(in_points[i], line_SI[i][0], line_SI[i][1]); // 0 -> left, 1-> middle, 2 -> right
            // Logger(BLUE, "in_point[%d]", i);
        }

        y_target = frame.rows - 220;
        x_target = ((y_target - line_SI[1][1]) / line_SI[1][0]);

        prev_left[0] = 0;
        prev_middle[0] = 1;
        prev_right[0] = 2;
        prev_left[1] = ((y_target - line_SI[0][1]) / line_SI[0][0]);
        prev_middle[1] = ((y_target - line_SI[1][1]) / line_SI[1][0]);
        prev_right[1] = ((y_target - line_SI[2][1]) / line_SI[2][0]);
    } else if (spike_final > 3 && !must3Lines) {
        Logger(GREEN, "MORE THAN 3 LINES DETECTED");
        for (int i = 0; i < in_points.size(); i++) {
            // checking if prev x is in one of new sliding windows then assign new index of sliding windows become road target(middle)
            if (abs(prev_x_target - in_points[i][0]) < 100 && abs(prev_x_target - in_points[i][2]) < 100) {
                road_target = i;
                Logger(YELLOW, "New Road Target %d", i);
                problem = false;
            }
            if (abs(prev_left[1] - in_points[i][0]) <= 100 && abs(prev_left[1] - in_points[i][2]) <= 100) {
                prev_left[0] = i;
                SlopeIntercept(in_points[i], line_SI[0][0], line_SI[0][1]); // left
            } else if (abs(prev_middle[1] - in_points[i][0]) <= 100 && abs(prev_middle[1] - in_points[i][2]) <= 100) {
                prev_middle[0] = i;
                SlopeIntercept(in_points[i], line_SI[1][0], line_SI[1][1]); // middle
            } else if (abs(prev_right[1] - in_points[i][0]) <= 100 && abs(prev_right[1] - in_points[i][2]) <= 100) {
                prev_right[0] = i;
                SlopeIntercept(in_points[i], line_SI[2][0], line_SI[2][1]); // right
            }
        }

        Logger(GREEN, "left : %d | mid : %d | right : %d", prev_left[0], prev_middle[0], prev_right[0]);

        prev_left[1] = ((y_target - line_SI[0][1]) / line_SI[0][0]);
        prev_middle[1] = ((y_target - line_SI[1][1]) / line_SI[1][0]);
        prev_right[1] = ((y_target - line_SI[2][1]) / line_SI[2][0]);

        // if above cannot determine new road target from prev x in new sliding windows, problem cannot be false
        if (problem) {
            Logger(RED, "SOMETHING WRONG IN ROAD TARGET");
            Logger(YELLOW, "current road target %d", road_target);
            Logger(YELLOW, "current prev x target %d", prev_x_target);
            Logger(YELLOW, "current prev spike %d", prev_spike);
            x_target = prev_x_target;
            y_target = frame.rows - 220;
        }
    } else if (spike_final == 2 && (!must3Lines)) {
        Logger(GREEN, "2 LINES DETECTED");
        for (int i = 0; i < in_points.size(); i++) {
            // checking if prev x is in one of new sliding windows then assign new index of sliding windows become road target(middle)
            if (abs(prev_x_target - in_points[i][0]) <= 100 && abs(prev_x_target - in_points[i][2]) <= 100) {
                road_target = i;
                Logger(YELLOW, "New Road Target %d", i);
                problem = false;
                break;
            }
        }

        if (road_target = 0) {
            SlopeIntercept(in_points[0], line_SI[1][0], line_SI[1][1]); // middle
            SlopeIntercept(in_points[1], line_SI[2][0], line_SI[2][1]); // right
        } else {
            SlopeIntercept(in_points[0], line_SI[0][0], line_SI[0][1]); // left
            SlopeIntercept(in_points[1], line_SI[1][0], line_SI[1][1]); // middle
        }

        y_target = frame.rows - 220;
        x_target = ((y_target - line_SI[1][1]) / line_SI[1][0]);
        if (line_SI[0][0] != 0) {
            prev_left[0] = road_target - 1;
            prev_left[1] = ((y_target - line_SI[0][1]) / line_SI[0][0]);
        }
        if (line_SI[1][0] != 0) {
            prev_middle[0] = road_target;
            prev_middle[1] = ((y_target - line_SI[1][1]) / line_SI[1][0]);
        }
        if (line_SI[2][0] != 0) {
            prev_right[0] = road_target + 1;
            prev_right[1] = ((y_target - line_SI[2][1]) / line_SI[2][0]);
        }

        // if above cannot determine new road target from prev x in new sliding windows, problem cannot be false
        if (problem) {
            Logger(RED, "SOMETHING WRONG IN ROAD TARGET");
            Logger(YELLOW, "current road target %d", road_target);
            Logger(YELLOW, "current prev x target %d", prev_x_target);
            Logger(YELLOW, "current prev spike %d", prev_spike);
            x_target = prev_x_target;
            y_target = frame.rows - 220;
            // must3Lines = true;
            // canbeIntercept = false;
        }
    } else if (spike_final < 2 && (!must3Lines)) {
        Logger(GREEN, "1 LINE DETECTED");
        if (abs(prev_x_target - in_points[0][0]) < 100 && abs(prev_x_target - in_points[0][2]) < 100) {
            road_target = 0;
            Logger(YELLOW, "New Road Target %d", 0);
            problem = false;
        }

        if (road_target = 0) {
            SlopeIntercept(in_points[0], line_SI[1][0], line_SI[1][1]); // middle
        }

        Logger(BLUE, "ROAD TARGET < 2");
        Logger(GREEN, "current road target %d", road_target);
        Logger(GREEN, "current prev x target %d", prev_x_target);
        Logger(GREEN, "current prev spike %d", prev_spike);
        x_target = prev_x_target;
        y_target = frame.rows - 220;
        must3Lines = true;
        canbeIntercept = false;
    }

    if (find3Lines && !must3Lines) {
        must3Lines = true;
    }
    // printf("x_target : %.f %.f || %.f %.f || %.f\n", x_target, y_target, line_SI[1][1], line_SI[1][0], (y_target - line_SI[1][1]) / line_SI[1][0]);
    if (spike_final > 0) {
        // problem false indicating there is no problem in assigning road target
        //  if (!problem) {
        //      y_target = frame.rows - 220;
        //      x_target = (int)((y_target - line_SI[road_target][1]) / line_SI[road_target][0]);
        //  } else {
        //      problem = false;
        //  }
        //  Logger(BLUE, "x1 : %d y1 : %d", x_target, y_target);

        // for (int i = 0; i < 3; i++) {
        //     Logger(BLUE, "slope[%d] : %f|| intercept[%d] : %f", i, line_SI[i][0], i, line_SI[1][1]);
        // }

        cv::circle(frame_dst, cv::Point(prev_left[1], y_target), 5, cv::Scalar(255, 0, 0), 12);
        cv::circle(frame_dst, cv::Point(prev_middle[1], y_target), 5, cv::Scalar(0, 255, 0), 12);
        cv::circle(frame_dst, cv::Point(prev_right[1], y_target), 5, cv::Scalar(0, 0, 255), 12);

        std::vector<cv::Vec4i> line_left = MakePoints(frame_dst, cv::Vec2f(line_SI[0][0], line_SI[0][1]));
        std::vector<cv::Vec4i> line_mid = MakePoints(frame_dst, cv::Vec2f(line_SI[1][0], line_SI[1][1]));
        std::vector<cv::Vec4i> line_right = MakePoints(frame_dst, cv::Vec2f(line_SI[2][0], line_SI[2][1]));

        Display(frame_dst, line_left, 225, 0, 0, 0.5);
        Display(frame_dst, line_mid, 0, 255, 0, 0.5);
        Display(frame_dst, line_right, 0, 0, 255, 0.5);

        if (!isnan(x_target)) {
            prev_x_target = x_target;
            target_x = x_target;
        }

        target_y = y_target;

        cv::circle(frame_dst, cv::Point(target_x, target_y), 3, cv::Scalar(255, 255, 0), 7);
        // cv::circle(frame_dst, cv::Point(x_target_right, y_target_right), 3, cv::Scalar(0, 255, 255), 7);

        msg_collection::RealPosition lane;
        float dist_x = 800 - target_y;
        float dist_y = target_x - 400;

        float distance = pixel_to_real(sqrt(pow(dist_x, 2) + pow(dist_y, 2)));
        float angle_diff = atan(dist_x / dist_y);

        if (dist_y < 0)
            angle_diff += DEG2RAD(180);

        // printf("angle %f dist %f\n", RAD2DEG(angle_diff), distance);

        if (prev_left[0] != 6969) {
            lane.left_lane_x_top = in_points[prev_left[0]][0];
            lane.left_lane_y_top = in_points[prev_left[0]][1];
            lane.left_lane_x_bottom = in_points[prev_left[0]][2];
            lane.left_lane_y_bottom = in_points[prev_left[0]][3];
        }

        if (prev_middle[0] != 6969) {
            lane.middle_lane_x_top = in_points[prev_middle[0]][0];
            lane.middle_lane_y_top = in_points[prev_middle[0]][1];
            lane.middle_lane_x_bottom = in_points[prev_middle[0]][2];
            lane.middle_lane_y_bottom = in_points[prev_middle[0]][3];
        }

        if (prev_right[0] != 6969) {
            lane.right_lane_x_top = in_points[prev_right[0]][0];
            lane.right_lane_y_top = in_points[prev_right[0]][1];
            lane.right_lane_x_bottom = in_points[prev_right[0]][2];
            lane.right_lane_y_bottom = in_points[prev_right[0]][3];
        }

        lane.target_x = distance * sin(angle_diff);
        lane.target_y = distance * cos(angle_diff);

        lane.can_be_intercepted = canbeIntercept;

        // Logger(GREEN, "x_pub : %f y_pub : %f", lane.target_x_left, lane.target_y_left);

        // printf("bef %f %f || nnnn %f %f\n", dist_x_left, dist_y_left, lane.target_x_left, lane.target_y_left);

        pub_target.publish(lane);
    }
}

void CenterSpike(cv::Mat frame, int start, int stop, int& index)
{
    int a = 0;
    int b = 0;
    int temp = 0;
    for (int i = start; i <= stop; i++) {
        a += frame.at<int>(0, i) * i;
        b += frame.at<int>(0, i);

        if (frame.at<int>(0, i) > temp) {
            temp = frame.at<int>(0, i);
            index = i;
        }
    }
    // index = (int)(a / (float)(b));
    // Logger(GREEN, "center x : %d", index);
}

void SlopeIntercept(cv::Vec4i& lines, double& slope, double& intercept)
{
    double x1 = lines[0];
    double y1 = lines[1];
    double x2 = lines[2];
    double y2 = lines[3];

    slope = (y2 - y1) / (x2 - x1);
    intercept = y1 - (slope * x1);
}