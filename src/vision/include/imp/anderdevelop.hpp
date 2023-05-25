#include "opencv2/opencv.hpp"

using namespace cv;

Mat raw_frame;

// Mat K = (Mat_<double>(3, 3) << 476.7030836014194, 0.0, 400.5, 0.0, 476.7030836014194, 400.5, 0.0, 0.0, 1.0);
Mat K = (Mat_<double>(3, 4) << 476.7030836014194, 0.0, 400.5, 0.0,
         0.0, 476.7030836014194, 400.5, 0.0,
         0.0, 0.0, 1.0, 0.0);
// Mat R = (Mat_<double>(3, 3) << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
Mat R = (Mat_<double>(4, 4) << 1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, -0.0, 0.0,
         0.0, 0.0, 1.0, 0.0,
         0.0, -0.0, 0.0, 0.0);
Mat P = (Mat_<double>(3, 4) << 476.7030836014194, 0.0, 400.5, -33.36921585209936, 0.0, 476.7030836014194, 400.5, 0.0, 0.0, 0.0, 1.0, 0.0);

// matrix projection 2D � 3D
Mat A1 = (Mat_<double>(4, 3) << 1.0, 0.0, -400.5, 0.0, 1.0, -400.5, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0);

// T - translation matrix
Mat T = (Mat_<double>(4, 4) << 1.0, 0.0, 0.0, -0.75, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0, -2.025, 0.0, 0.0, 0.0, 1.0);

Mat map1, map2;
