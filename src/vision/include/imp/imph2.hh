#ifndef __IMP_HH_
#define __IMP_HH_

#include <iostream>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <math.h>

using namespace cv;

#define CAMERA_POS_X 0.75
#define CAMERA_POS_Y 0.0
#define CAMERA_POS_Z 2.025
#define FOCAL_LENGTH 476.703
#define PRINCIPAL_POINT 400.5
#define FRAME_HEIGHT 800
#define FRAME_WIDTH 800
#define PI 3.14159265358979323846

int alpha_ = 90;
int beta_ = 90;
int gamma_ = 90;
int dist_ = 500;
int image_h = 223;
int image_w = 900;

Rect Rec(0, 410, 800, 390);

Mat source;
Mat dest;

#endif // __IMP_HH_