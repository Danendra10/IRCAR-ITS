#include "vision/urban.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_urban");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    ros::MultiThreadedSpinner spinner;

    // namedWindow("slider_thresh", WINDOW_NORMAL);
    // createTrackbar("h_min_lane", "slider_thresh", &h_min_lane, 180);
    // createTrackbar("h_max_lane", "slider_thresh", &h_max_lane, 180);
    // createTrackbar("s_min_lane", "slider_thresh", &s_min_lane, 255);
    // createTrackbar("s_max_lane", "slider_thresh", &s_max_lane, 255);
    // createTrackbar("v_min_lane", "slider_thresh", &v_min_lane, 255);
    // createTrackbar("v_max_lane", "slider_thresh", &v_max_lane, 255);
    // createTrackbar("h_min_road", "slider_thresh", &h_min_road, 180);
    // createTrackbar("h_max_road", "slider_thresh", &h_max_road, 180);
    // createTrackbar("s_min_road", "slider_thresh", &s_min_road, 255);
    // createTrackbar("s_max_road", "slider_thresh", &s_max_road, 255);
    // createTrackbar("v_min_road", "slider_thresh", &v_min_road, 255);
    // createTrackbar("v_max_road", "slider_thresh", &v_max_road, 255);

    pub_error_angle = nh.advertise<std_msgs::Float32>("/vision/error_angle", 1);

    sub_raw_frame = it.subscribe("/catvehicle/camera_front/image_raw_front", 1, CllbckSubFrame);

    tim_30hz = nh.createTimer(ros::Duration(1.0 / 30.0), CllbckTim30Hz);

    spinner.spin();
}

Mat DetectSpikeThreshold(Mat frame_thresholded)
{
    Mat frame_thresholded_blur;
    GaussianBlur(frame_thresholded, frame_thresholded_blur, Size(5, 5), 0);

    Mat frame_thresholded_blur_thresh;
    threshold(frame_thresholded_blur, frame_thresholded_blur_thresh, 100, 255, THRESH_BINARY);

    return frame_thresholded_blur_thresh;
}

void ClickFrame(Mat *frame)
{
    // write position of mouse click to console and draw a circle and write coordinates
    setMouseCallback("frame", [](int event, int x, int y, int flags, void *userdata)
                     {
        if (event == EVENT_LBUTTONDOWN)
        {
            cout << "x: " << x << " y: " << y << endl;
            circle(*((Mat *)userdata), Point(x, y), 5, Scalar(0, 0, 255), -1);
            putText(*((Mat *)userdata), to_string(x) + ", " + to_string(y), Point(x, y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1);
        } });
}

Point VectorPointMean(vector<Point> vec_1, vector<Point> vec_2)
{
    float mean_x = 0;
    float mean_y = 0;
    for (int i = 0; i < vec_1.size(); i++)
    {
        mean_x += vec_1[i].x;
        mean_y += vec_1[i].y;
    }
    for (int i = 0; i < vec_2.size(); i++)
    {
        mean_x += vec_2[i].x;
        mean_y += vec_2[i].y;
    }
    mean_x /= (vec_1.size() + vec_2.size());
    mean_y /= (vec_1.size() + vec_2.size());
    return Point(mean_x, mean_y);
}

void CllbckTim30Hz(const ros::TimerEvent &)
{
    if (validators != 0b001)
        return;

    mutex_raw_frame.lock();
    Mat frame = raw_frame.clone();
    mutex_raw_frame.unlock();

    //     x: 781 y: 576
    // x: 377 y: 413
    // x: 267 y: 413
    // x: 9 y: 468
    // x: 1 y: 668

    vector<Point>
        roi_points = {
            Point(799, 576), // top right
            Point(377, 413), // bottom right
            Point(267, 413), // bottom left
            Point(2, 468),   // top left
            Point(2, 668)    // top left
        };

    // Create a binary mask image for the ROI
    Mat roi_mask = Mat::zeros(frame.size(), CV_8UC1);

    // Fill the ROI region with white pixels
    fillPoly(roi_mask, vector<vector<Point>>{roi_points}, Scalar(255));

    Mat frame_hsv;
    cvtColor(frame, frame_hsv, COLOR_BGR2HSV);

    Mat frame_hsv_roi;
    frame_hsv.copyTo(frame_hsv_roi, roi_mask);

    Mat road_frame = frame_hsv_roi.clone();
    inRange(road_frame, Scalar(h_min_road, s_min_road, v_min_road), Scalar(h_max_road, s_max_road, v_max_road), frame_thresholded_road);

    // road contour
    vector<vector<Point>> contours_road;
    vector<Vec4i> hierarchy_road;
    findContours(frame_thresholded_road, contours_road, hierarchy_road, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

    inRange(frame_hsv_roi, Scalar(h_min_lane, s_min_lane, v_min_lane), Scalar(h_max_lane, s_max_lane, v_max_lane), frame_thresholded_lane);
    Mat lane_or_field;
    vector<vector<Point>> contours_lane_or_field;
    vector<Vec4i> hierarchy_lane_or_field;
    bitwise_or(frame_thresholded_lane, frame_thresholded_road, lane_or_field);
    bitwise_and(lane_or_field, frame_thresholded_lane, lane_or_field);
    findContours(lane_or_field, contours_lane_or_field, hierarchy_lane_or_field, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

    // canny and hough
    Mat canny_output;
    Canny(frame_thresholded_lane, canny_output, 100, 100 * 2, 3);
    vector<Vec4i> lines;
    HoughLinesP(canny_output, lines, 1, CV_PI / 180, 50, 50, 10);

    // draw contours
    Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3);
    for (int i = 0; i < contours_lane_or_field.size(); i++)
    {
        Scalar color = Scalar(255, 255, 255);
        drawContours(drawing, contours_lane_or_field, i, color, 2, 8, hierarchy_lane_or_field, 0, Point());
    }

    // find the middle of the road based on the lines, find the left and right lane
    vector<Point> left_lane;
    vector<Point> right_lane;

    for (int x = 0; x < drawing.cols; x++)
    {
        for (int y = 0; y < drawing.rows; y++)
        {
            if (drawing.at<Vec3b>(Point(x, y)) == Vec3b(255, 255, 255))
            {
                left_lane.push_back(Point(x, y));
                break;
            }
        }
    }

    // iterate from right to left the first line that is found is the right lane
    for (int x = drawing.cols - 1; x >= drawing.cols / 2; x--)
    {
        for (int y = 0; y < drawing.rows; y++)
        {
            if (drawing.at<Vec3b>(Point(x, y)) == Vec3b(255, 255, 255))
            {
                right_lane.push_back(Point(x, y));
                break;
            }
        }
    }

    Point final_middle = VectorPointMean(left_lane, right_lane);

    int middle_cam_x = drawing.cols / 2;
    int middle_cam_y = drawing.rows;

    int middle_x = final_middle.x;
    int middle_y = final_middle.y;

    double distance = sqrt(pow(middle_cam_x - middle_x, 2) + pow(middle_cam_y - middle_y, 2));
    double angle = atan2(middle_cam_y - middle_y, middle_cam_x - middle_x) * 180 / M_PI - 90;

    line(drawing, Point(middle_cam_x, middle_cam_y), Point(middle_x, middle_y), Scalar(0, 0, 255), 2);

    putText(drawing, to_string(angle), Point(10, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
    putText(drawing, to_string(distance), Point(10, 100), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);

    circle(drawing, Point(final_middle.x, final_middle.y), 5, Scalar(0, 0, 255), -1);

    // middle line
    line(drawing, Point(drawing.cols / 2, 0), Point(drawing.cols / 2, drawing.rows), Scalar(0, 0, 255), 2);

    std_msgs::Float32 angle_error_msg;
    angle_error_msg.data = angle;
    pub_error_angle.publish(angle_error_msg);

    imshow("frame", frame);
    // print the coordinates of the mouse click
    setMouseCallback(
        "frame",
        [](int event, int x, int y, int flags, void *userdata)
        {
            if (event == EVENT_LBUTTONDOWN)
            {
                cout << "x: " << x << " y: " << y << endl;
            }
        },
        NULL);
    imshow("drawing", drawing);
    waitKey(1);
}