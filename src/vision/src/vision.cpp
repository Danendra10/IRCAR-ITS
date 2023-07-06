/**
 * @copyright @Danendra10 & @isabellejt
 * @brief This node will calculate the line of the road
 *       and publish the point of the line
 * @license IRIS
 * TODO: interpolate the detected lanes @Danendra10
 */

#include "vision/vision.hh"
int a, b, c;

int main(int argc, char **argv)
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

    pub_car_pose = NH.advertise<geometry_msgs::Point>("/car_pose", 1);
    pub_points = NH.advertise<msg_collection::PointArray>("/lines", 1);
    pub_target = NH.advertise<msg_collection::RealPosition>("/real_lines", 1);

    MTS.spin();

    return 0;
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
    car_pose.th = RAD2DEG(2 * atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));

    geometry_msgs::Point car_pose_msg;
    car_pose_msg.x = car_pose.x;
    car_pose_msg.y = car_pose.y;
    car_pose_msg.z = car_pose.th;
    pub_car_pose.publish(car_pose_msg);
    // ROS_ERROR("z = %f || w = %f || th = %f", msg->pose.pose.orientation.z, msg->pose.pose.orientation.w, car_pose.th);
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

        // if (i % 10 == 0)
        // printf("obs %f %f || dist %f\n", msg->x[i], raw_obstacle->x, raw_obstacle->dist);
        // printf("obs %f %f || dist %f\n", raw_obstacle->x, raw_obstacle->y, raw_obstacle->dist);
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

    for (int i = 0; i < left_lane.size(); i++)
    {
        circle(lane_points, left_lane[i], 3, Scalar(255, 0, 0), -1);
        lane.left_lane_x.push_back(left_lane[i].x);
        lane.left_lane_y.push_back(left_lane[i].y);
    }

    vector<Point> right_lane = detect.getRightLane();

    for (int i = 0; i < right_lane.size(); i++)
    {
        circle(lane_points, right_lane[i], 3, Scalar(0, 255, 0), -1);
        lane.right_lane_x.push_back(right_lane[i].x);
        lane.right_lane_y.push_back(right_lane[i].y);
    }

    vector<Point> middle_lane = detect.calcMiddleLane();
    vector<double> x_middle_lane;
    vector<double> y_middle_lane;

    for (int i = 0; i < middle_lane.size(); i++)
    {
        circle(lane_points, middle_lane[i], 3, Scalar(255, 255, 255), -1);
        lane.middle_lane_x.push_back(middle_lane[i].x);
        lane.middle_lane_y.push_back(middle_lane[i].y);
        x_middle_lane.push_back(middle_lane[i].x);
        y_middle_lane.push_back(middle_lane[i].y);
    }

    pub_points.publish(lane);

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

    // Detect(line_bgr);
    Detect(raw_frame);

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

    for (int i = 0; i < edges.rows; i++)
    {
        for (int j = 0; j < edges.cols; j++)
        {
            if (edges.at<uchar>(i, j) == 255)
            {
                points.push_back(Point(j, i));
            }
        }
    }

    return points;
}

std::vector<cv::Vec4i> GetLeftLines(const std::vector<cv::Vec4i> &lines)
{
    std::vector<cv::Vec4i> left_lines;
    int min_x = 800; // Initialize with a large value

    for (const cv::Vec4i &line : lines)
    {
        cv::Point p1(line[0], line[1]);
        cv::Point p2(line[2], line[3]);

        // Check if the line is within the left region
        if (p1.x <= min_x && p2.x <= min_x && p1.x <= 300 && p2.x <= 300)
        {
            // Check if the line is close to the previous line
            if (left_lines.empty() || std::abs(p1.x - min_x) <= 50)
            {
                left_lines.push_back(line);
                min_x = std::min(p1.x, p2.x);
            }
        }
    }

    return left_lines;
}

std::vector<cv::Vec4i> GetRightLines(const std::vector<cv::Vec4i> &lines, int frameWidth)
{
    std::vector<cv::Vec4i> right_lines;
    int max_x = 0; // Initialize with a small value

    for (const cv::Vec4i &line : lines)
    {
        cv::Point p1(line[0], line[1]);
        cv::Point p2(line[2], line[3]);

        // Check if the line is within the right region
        if (p1.x >= max_x && p2.x >= max_x && p1.x >= frameWidth - 300 && p2.x >= frameWidth - 300)
        {
            // Check if the line is close to the previous line
            if (right_lines.empty() || std::abs(p1.x - max_x) <= 50)
            {
                right_lines.push_back(line);
                max_x = std::max(p1.x, p2.x);
            }
        }
    }

    return right_lines;
}

std::vector<cv::Vec4i> GetMiddleLines(const std::vector<cv::Vec4i> &lines, int frameWidth)
{
    std::vector<cv::Vec4i> middle_lines;
    int min_x = frameWidth / 2 - 100; // Left boundary of middle region
    int max_x = frameWidth / 2 + 100; // Right boundary of middle region

    for (const cv::Vec4i &line : lines)
    {
        cv::Point p1(line[0], line[1]);
        cv::Point p2(line[2], line[3]);

        // Check if the line is within the middle region
        if ((p1.x >= min_x && p1.x <= max_x) || (p2.x >= min_x && p2.x <= max_x))
        {
            if (middle_lines.empty() || std::abs(p1.x - p2.x) <= 50)
            {
                middle_lines.push_back(line);
            }
        }
    }

    return middle_lines;
}

std::vector<cv::Vec4i> GetMiddlePoints(const std::vector<cv::Vec4i> &leftLines, const std::vector<cv::Vec4i> &middleLines)
{
    std::vector<cv::Vec4i> middlePoints;

    for (size_t i = 0; i < leftLines.size() && i < middleLines.size(); ++i)
    {
        cv::Point2f leftLineStart(leftLines[i][0], leftLines[i][1]);
        cv::Point2f leftLineEnd(leftLines[i][2], leftLines[i][3]);

        cv::Point2f middleLineStart(middleLines[i][0], middleLines[i][1]);
        cv::Point2f middleLineEnd(middleLines[i][2], middleLines[i][3]);

        cv::Point2f middlePt = 0.5f * (leftLineStart + middleLineStart);

        middlePoints.push_back(cv::Vec4i(middlePt.x, middlePt.y));
    }

    return middlePoints;
}

cv::Vec4i ExtrapolateLine(const cv::Vec4i &line, int minY, int maxY)
{
    double slope = static_cast<double>(line[3] - line[1]) / static_cast<double>(line[2] - line[0]);
    int startX = line[0] + static_cast<int>((minY - line[1]) / slope);
    int endX = line[0] + static_cast<int>((maxY - line[1]) / slope);
    return cv::Vec4i(startX, minY, endX, maxY);
}

std::vector<cv::Point> GetLeftPoints(const std::vector<cv::Point> &points)
{
    std::vector<cv::Point> left_points;
    int min_x = 800; // Initialize with a large value

    for (const cv::Point &point : points)
    {
        if (point.y < 100)
            continue;
        // Check if the point is within the left region
        if (point.x <= min_x && point.x <= 300)
        {
            // Check if the point is close to the previous point
            if (left_points.empty() || std::abs(point.x - min_x) <= 50)
            {
                left_points.push_back(point);
                min_x = point.x;
            }
        }
    }

    return left_points;
}

std::vector<cv::Point> GetRightPoints(const std::vector<cv::Point> &points, int frameWidth)
{
    std::vector<cv::Point> right_points;
    int max_x = 0; // Initialize with a small value

    for (const cv::Point &point : points)
    {
        if (point.y < 100)
            continue;
        // Check if the point is within the right region
        if (point.x >= max_x && point.x >= frameWidth - 300)
        {
            // Check if the point is close to the previous point
            if (right_points.empty() || std::abs(point.x - max_x) <= 50)
            {
                right_points.push_back(point);
                max_x = point.x;
            }
        }
    }

    return right_points;
}

std::vector<cv::Point> GetMiddlePoints(const std::vector<cv::Point> &points, int frameWidth)
{
    std::vector<cv::Point> middle_points;
    int min_x = frameWidth / 2 - 100; // Left boundary of middle region
    int max_x = frameWidth / 2 + 100; // Right boundary of middle region

    for (const cv::Point &point : points)
    {
        if (point.y < 100)
            continue;
        // check if the point exist in the left and right region
        for (const cv::Point &left_point : left_points)
        {
            if (point.x == left_point.x && point.y == left_point.y)
            {
                continue;
            }
        }
        for (const cv::Point &right_point : right_points)
        {
            if (point.x == right_point.x && point.y == right_point.y)
            {
                continue;
            }
        }
        if (point.x >= min_x && point.x <= max_x)
        {

            if (middle_points.empty() || std::abs(point.x - point.x) <= 50)
            {
                middle_points.push_back(point);
            }
        }
    }

    return middle_points;
}

std::vector<cv::Point> GetMiddleOfLeftRoad(const std::vector<cv::Point> &leftPoints, const std::vector<cv::Point> &middlePoints)
{
    std::vector<cv::Point> middleOfLeftRoad;

    for (size_t i = 0; i < leftPoints.size() && i < middlePoints.size(); ++i)
    {
        cv::Point2f leftPoint(leftPoints[i].x, leftPoints[i].y);
        cv::Point2f middlePoint(middlePoints[i].x, middlePoints[i].y);

        cv::Point2f middlePt = 0.5f * (leftPoint + middlePoint);

        middleOfLeftRoad.push_back(middlePt);
    }

    return middleOfLeftRoad;
}

std::vector<cv::Point> GetMiddleOfRightRoad(const std::vector<cv::Point> &rightPoints, const std::vector<cv::Point> &middlePoints)
{
    std::vector<cv::Point> middleOfRightRoad;

    for (size_t i = 0; i < rightPoints.size() && i < middlePoints.size(); ++i)
    {
        cv::Point2f rightPoint(rightPoints[i].x, rightPoints[i].y);
        cv::Point2f middlePoint(middlePoints[i].x, middlePoints[i].y);

        cv::Point2f middlePt = 0.5f * (rightPoint + middlePoint);

        middleOfRightRoad.push_back(middlePt);
    }

    return middleOfRightRoad;
}

Mat DrawObsPoints(const vector<ObstaclesPtr> &points)
{
    Mat frame = Mat::zeros(800, 800, CV_8UC3);
    // draw car in the middle
    float car_x = 400;
    float car_y = 700;

    cv::circle(frame, cv::Point(car_x, car_y), 30, cv::Scalar(0, 255, 0), 2);

    for (int i = 0; i < points.size(); i++)
    {
        float obs_y = (700 - points[i]->x * 30);
        float obs_x = (points[i]->y * 60 + 400);

        // printf("obs frame x %.2f y %.2f\n", obs_x, obs_y);
        // printf("points obs x %.2f y %.2f\n", points[i]->x, points[i]->y);
        cv::circle(frame, cv::Point(obs_x, obs_y), 5, cv::Scalar(0, 0, 255), -1);
    }

    for (int i = 0; i < middle_points.size(); i++)
    {
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
    cv::Mat frame_gray, frame_canny;
    cv::Mat result = frame.clone();
    std::vector<cv::Vec4i> line_hough;

    cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(frame_gray, frame_gray, cv::Size(5, 5), 0);
    cv::Canny(frame_gray, frame_canny, 25, 50);
    ROI(frame_canny);
    Hough(frame_canny, line_hough);
    // SlidingWindows(result, line_hough);
    Display(result, line_hough, 0, 255, 0, 0.5);
    Average(result, line_hough);
    Display(result, line_hough, 255, 255, 255, 0.5);
    // cv::imshow("edge", frame_canny);
    setMouseCallback("result", click_event);

    cv::imshow("result", result);
}

void ROI(cv::Mat &frame)
{
    cv::Mat frame_mask(frame.rows, frame.cols, CV_8UC1, cv::Scalar(0));
    std::vector<cv::Point> ROI;

    // ROI.push_back(cv::Point(200, frame.rows));
    // ROI.push_back(cv::Point(1100, frame.rows));
    // ROI.push_back(cv::Point(555, 290));
    // ROI.push_back(cv::Point(545, 290));

    ROI.push_back(cv::Point(0, frame.rows / 2 + 40));          // top left
    ROI.push_back(cv::Point(frame.cols, frame.rows / 2 + 40)); // top right
    ROI.push_back(cv::Point(frame.cols, frame.rows - 122));    // bottom right
    ROI.push_back(cv::Point(0, frame.rows - 122));             // bottom left

    std::vector<cv::Point> Ignore;

    Ignore.push_back(cv::Point(0, frame.rows - 122));
    Ignore.push_back(cv::Point(frame.cols / 2 - 200, frame.rows / 2 + 70));
    Ignore.push_back(cv::Point(frame.cols / 2 + 50, frame.rows / 2 + 70));
    Ignore.push_back(cv::Point(2 * frame.cols / 3 + 100, frame.rows - 122));

    fillConvexPoly(frame_mask, ROI, cv::Scalar(255));
    fillConvexPoly(frame_mask, Ignore, cv::Scalar(0));
    // cv::rectangle(frame_mask, cv::Point(245, 795), cv::Point(555, 636), cv::Scalar(0), CV_FILLED);
    cv::bitwise_and(frame, frame_mask, frame);

    // cv::imshow("mask", frame_mask);
}

void Hough(cv::Mat frame, std::vector<cv::Vec4i> &line)
{
    cv::HoughLinesP(frame, line, 2, CV_PI / 180, 94, 36, 14); // 100,115,15
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

void Average(cv::Mat frame, std::vector<cv::Vec4i> &lines)
{
    std::vector<cv::Vec2f> left_fit;
    std::vector<cv::Vec2f> right_fit;
    int temp = 0;

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

    for (size_t i = 0; i < lines.size(); i++)
    {
        double x1 = lines[i][0];
        double y1 = lines[i][1];
        double x2 = lines[i][2];
        double y2 = lines[i][3];

        double slope = (y2 - y1) / (x2 - x1);
        double intercept = y1 - (slope * x1);

        // Print the coefficients of the fitted polynomial
        // std::cout << "Slope: " << slope << std::endl;
        // std::cout << "Intercept: " << coeffs[1] << std::endl;

        // std::cout << x1 << " " << x2 <<std::endl;
        if (slope < 0 && x1 < 350 && x2 < 350)
        {
            left_fit.push_back(cv::Vec2f(slope, intercept));
        }
        else if (slope > 0 && x1 > 350 && x2 > 350)
        {
            right_fit.push_back(cv::Vec2f(slope, intercept));
        }
    }

    cv::Vec2f left_fit_avg = VectorAvg(left_fit);
    cv::Vec2f right_fit_avg = VectorAvg(right_fit);

    // std::cout<<"Left : "<<left_fit_avg<<std::endl;
    // std::cout<<"Right : "<<right_fit_avg<<std::endl;

    std::vector<cv::Vec4i> line_left = MakePoints(frame, left_fit_avg);
    std::vector<cv::Vec4i> line_right = MakePoints(frame, right_fit_avg);
    std::vector<cv::Vec4i> line_mid, line_target;

    // std::cout<<line_left[0]<<" "<<line_right[0]<<std::endl;
    if (!std::isnan(left_fit_avg[0]) && !std::isnan(right_fit_avg[0]))
    {
        int x1 = (line_left[0][0] + line_right[0][0]) / 2.0;
        int y1 = (line_left[0][1] + line_right[0][1]) / 2.0;
        int x2 = (line_left[0][2] + line_right[0][2]) / 2.0;
        int y2 = (line_left[0][3] + line_right[0][3]) / 2.0;
        line_mid.push_back(cv::Vec4i(x1, y1, x2, y2));

        int mid_x1 = (x1 + line_right[0][0]) / 2.0;
        int mid_y1 = (y1 + line_right[0][1]) / 2.0;
        int mid_x2 = (x2 + line_right[0][2]) / 2.0;
        int mid_y2 = (y2 + line_right[0][3]) / 2.0;
        line_target.push_back(cv::Vec4i(mid_x1, mid_y1, mid_x2, mid_y2));

        target_x = (mid_x1 + mid_x2) / 2.0;
        target_y = (mid_y1 + mid_y2) / 2.0;

        cv::circle(frame, cv::Point(target_x, target_y), 5, cv::Scalar(255), 10);
        // std::cout<<target_x<<"  "<<target_y<<std::endl;
        msg_collection::RealPosition lane;
        lane.target_x = pixel_to_real(800 - target_y);
        lane.target_y = pixel_to_real(target_x - 400);
        printf("nnnn %.2f %.2f\n", lane.target_x, lane.target_y);
        pub_target.publish(lane);
    }
    else if (std::isnan(left_fit_avg[0]))
    {
        // ROS_WARN("KIRI HILANG");
        int x1 = line_right[0][0] / 2.0;
        int y1 = line_right[0][1];
        int x2 = line_right[0][2] / 2.0;
        int y2 = line_right[0][3];
        // std::cout<<x1<<" "<<x2<<std::endl;
        line_mid.push_back(cv::Vec4i(x1, y1, x2, y2));
    }
    else if (std::isnan(right_fit_avg[0]))
    {
        // ROS_WARN("KANAN HILANG");
        int x1 = line_left[0][0] + (abs(line_left[0][0]) / 2.0);
        int y1 = line_left[0][1];
        int x2 = line_left[0][2] + (abs(line_left[0][2]) / 2.0);
        int y2 = line_left[0][3];
        // std::cout<<x1<<" "<<x2<<std::endl;
        line_mid.push_back(cv::Vec4i(x1, y1, x2, y2));
    }
    else
    {
        // ROS_ERROR("WATEFAK");
    }

    Display(frame, line_left, 255, 0, 0, 1);
    Display(frame, line_right, 0, 0, 255, 1);
    Display(frame, line_mid, 255, 0, 255, 1);
    Display(frame, line_target, 0, 255, 255, 1);
}

cv::Vec2f VectorAvg(std::vector<cv::Vec2f> in_vec)
{
    float avg_x = 0;
    float avg_y = 0;
    for (int i = 0; i < in_vec.size(); i++)
    {
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

    return std::vector<cv::Vec4i>{cv::Vec4i(x1, y1, x2, y2)};
}

void SlidingWindows(cv::Mat frame, std::vector<Vec4i> lines)
{
    std::vector<cv::Rect> windows;
    const int numWindows = 20;
    const int windowWidth = 100;
    const int windowHeight = 20;
    int xMid[lines.size()];
    // int windowStep = (frame.rows/2) / numWindows;

    for (size_t i = 0; i < lines.size(); i++)
    {
        cv::Vec4i line = lines[i];
        xMid[i] = (line[0] + line[3]) / 2.0;
        // std::cout<<xMid[i]<<std::endl;
    }

    for (size_t i = 0; i < numWindows; i++)
    {
        int yTop = frame.rows - (i + 1) * windowHeight;
        int xLeft = xMid[i] - windowWidth / 2.0;

        cv::Rect window(xLeft, yTop, windowWidth, windowHeight);

        windows.push_back(window);
    }

    for (size_t i = 0; i < windows.size(); i++)
    {
        cv::Rect window = windows[i];
        cv::rectangle(frame, window, cv::Scalar(0, 255, 0), 1);
    }
}