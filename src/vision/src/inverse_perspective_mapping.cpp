#include <iostream>
#include <opencv2/opencv.hpp>

cv::Mat transformToBirdsEyeView(const cv::Mat &frame, const cv::Mat &K, const cv::Mat &R, const cv::Mat &T, int frameWidth, int frameHeight, int outputWidth, int outputHeight)
{
    // Step 1: Define the transformation matrix
    cv::Mat M = cv::Mat::zeros(3, 4, CV_64F);
    cv::hconcat(R, T, M);

    // Step 2: Define the projection matrix
    cv::Mat P = cv::Mat::zeros(3, 4, CV_64F);
    K.copyTo(P(cv::Rect(0, 0, 3, 3)));

    // Step 3: Compute the inverse projection matrix
    cv::Mat P_inv;
    cv::invert(P, P_inv, cv::DECOMP_LU);

    // Step 4: Generate bird's-eye view
    cv::Mat birdseyeView;
    cv::warpPerspective(frame, birdseyeView, P_inv * M, cv::Size(outputWidth, outputHeight));

    return birdseyeView;
}

int main()
{
    // Load your frame/image
    cv::Mat frame = cv::imread("path_to_your_frame_image.jpg");

    // Intrinsic camera parameters
    double fx = 476.7030836014194;
    double fy = 476.7030836014194;
    double cx = 400.5;
    double cy = 400.5;

    // Camera position
    double x = 0.75;
    double y = 0;
    double z = 2.025;

    // Calculate rotation matrix (R) and translation vector (T)
    cv::Mat R, T;
    cv::Rodrigues(cv::Vec3d(x, y, z), R);
    T = cv::Mat::zeros(3, 1, CV_64F);

    // Create the intrinsic matrix (K)
    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    K.at<double>(0, 0) = fx;
    K.at<double>(1, 1) = fy;
    K.at<double>(0, 2) = cx;
    K.at<double>(1, 2) = cy;

    // Specify the output dimensions of the bird's-eye view
    int outputWidth = 800;
    int outputHeight = 800;

    // Transform the frame to bird's-eye view
    cv::Mat birdseyeView = transformToBirdsEyeView(frame, K, R, T, frame.cols, frame.rows, outputWidth, outputHeight);

    // Display the bird's-eye view
    cv::imshow("Bird's-eye View", birdseyeView);
    cv::waitKey(0);

    return 0;
}
