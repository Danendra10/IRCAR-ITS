
#include <iostream>
#include <string>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

cv::Mat frame_raw;
cv::Mat frame_gray;
cv::Mat output_image = frame_raw.clone();

// aruco variables

// void ToGray()
// {
//     cv::cvtColor(frame_raw, frame_gray, cv::COLOR_BGR2GRAY);
// }

// void DetectMarkers()
// {
//     detector.detectMarkers(frame_gray, marker_corners, marker_ids, rejected_candidates);
// }

// void DrawMarkers()
// {
//     cv::aruco::drawDetectedMarkers(output_image, marker_corners, marker_ids);
// }

// void DebugMarkerParams()
// {
//     std::cout << "Marker IDs: " << marker_ids.size() << std::endl;
//     std::cout << "Marker Corners: " << marker_corners.size() << std::endl;
//     std::cout << "Rejected Candidates: " << rejected_candidates.size() << std::endl;
// }
