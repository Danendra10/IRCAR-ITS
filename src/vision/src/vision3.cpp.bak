/**
 * @copyright @Danendra10 & @isabellejt
 * @brief This node will calculate the line of the road
 *       and publish the point of the line
 * @license IRIS
 * TODO: interpolate the detected lanes @Danendra10
 */

#include "vision/vision.hh"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision");
    ros::NodeHandle NH;
    image_transport::ImageTransport IT(NH);
    ros::MultiThreadedSpinner MTS(0);

    Init();

    tim_30hz = NH.createTimer(ros::Duration(1.0 / 30.0), Tim30HzCllbck);

    sub_raw_frame = IT.subscribe("/catvehicle/camera_front/image_raw_front", 1, SubRawFrameCllbck);
    sub_odom = NH.subscribe("/catvehicle/odom", 1, SubOdomRaw);
    sub_lidar_data = NH.subscribe("/lidar_data", 1, SubLidarData);

    pub_car_pose = NH.advertise<geometry_msgs::Point>("/car_pose", 1);
    pub_points = NH.advertise<msg_collection::PointArray>("/lines", 1);

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
}

void SubLidarData(const msg_collection::Obstacles::ConstPtr &msg)
{
    obstacles.clear();
    raw_obstacles.clear();
    for (int i = 0; i < msg->x.size(); i++)
    {
        ObstaclesPtr obstacle(new Obstacles);
        obstacle->x = msg->x[i];
        obstacle->y = msg->y[i];
        obstacles.push_back(obstacle);

        ObstaclesPtr raw_obstacle(new Obstacles);
        raw_obstacle->x = msg->x[i] - car_pose.x;
        raw_obstacle->y = msg->y[i] - car_pose.y;
        raw_obstacles.push_back(raw_obstacle);
    }
}

void Tim30HzCllbck(const ros::TimerEvent &event)
{
    if (validators != 0b001)
        return;

    rectangle(raw_frame, Rec, Scalar(0, 0, 255), 2);
    source = raw_frame(Rec);

    resize(source, source, Size(FRAME_WIDTH, FRAME_HEIGHT));

    double focal_length, dist, alpha, beta, gamma, f;

    alpha = ((double)alpha_ - 90) * PI / 180;
    beta = ((double)beta_ - 90) * PI / 180;
    gamma = ((double)gamma_ - 90) * PI / 180;
    dist = (double)dist_;
    focal_length = (double)FOCAL_LENGTH;

    Size image_size = source.size();
    double w = (double)image_size.width, h = (double)image_size.height;

    Mat A1 = (Mat_<float>(4, 3) << 1, 0, -w / 2,
              0, 1, -h / 2,
              0, 0, 0,
              0, 0, 1);

    Mat RX = (Mat_<float>(4, 4) << 1, 0, 0, 0,
              0, cos(alpha), -sin(alpha), 0,
              0, sin(alpha), cos(alpha), 0,
              0, 0, 0, 1);

    Mat RY = (Mat_<float>(4, 4) << cos(beta), 0, -sin(beta), 0,
              0, 1, 0, 0,
              sin(beta), 0, cos(beta), 0,
              0, 0, 0, 1);

    Mat RZ = (Mat_<float>(4, 4) << cos(gamma), -sin(gamma), 0, 0,
              sin(gamma), cos(gamma), 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1);

    Mat R = RX * RY * RZ;

    Mat T = (Mat_<float>(4, 4) << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, dist,
             0, 0, 0, 1);

    Mat K = (Mat_<float>(3, 4) << focal_length, 0, w / 2, 0,
             0, focal_length, h / 2, 0,
             0, 0, 1, 0);

    Mat transformationMat = K * (T * (R * A1));

    warpPerspective(source, dest, transformationMat, image_size, INTER_CUBIC | WARP_INVERSE_MAP);

    imshow("Result", dest);

    imshow("Original", raw_frame);
    waitKey(1);
}

//========================================================================================================================

void Init()
{
    namedWindow("Original", 1);
    namedWindow("Result", 1);
    createTrackbar("Alpha", "Result", &alpha_, 180);
    createTrackbar("Beta", "Result", &beta_, 180);
    createTrackbar("Gamma", "Result", &gamma_, 180);
    createTrackbar("Distance", "Result", &dist_, 2000);
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

    cv::rectangle(frame, cv::Point(car_x - 20, car_y - 20), cv::Point(car_x + 20, car_y + 20), cv::Scalar(0, 255, 0), 2);

    for (int i = 0; i < points.size(); i++)
    {
        // float obs_x = (points[i]->y * 10) + 400;
        // float obs_y = (points[i]->x * -10) + 700;
        float obs_y = (points[i]->x * 8 + car_x);
        float obs_x = (points[i]->y * 8 + car_y);

        printf("obs frame x %.2f y %.2f\n", obs_x, obs_y);
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