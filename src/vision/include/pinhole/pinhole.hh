#include <cmath>
#include <iostream>
#include "opencv2/core/core.hpp"

using namespace std;
using namespace cv;

class CameraCalibrate
{
public:
    void NormPixel2Camera(float *x, float *y)
    {
        *x = (*x - this->cx) / this->fx;
        *y = (*y - this->cy) / this->fy;
    }

    // normed to 3d space
    void NormPixel2Space(float *x, float *y, float *z)
    {
        *x = (*x - this->cx) / this->fx;
        *y = (*y - this->cy) / this->fy;
        *z = 1.0f;
    }

    /**
     * Matrix of K is as follows:
     * --        --
     * | fx 0  cx |
     * | 0  fy cy |
     * | 0  0  1  |
     * --        --
     *
     * Matrix of K^-1 is as follows:
     * --                --
     * | 1/fx  0   -cx/fx |
     * | 0   1/fy  -cy/fy |
     * | 0    0       1   |
     * --                --
     *
     * The pixel point is (u, v) and the normed point is (x, y, z)
     * then it can be calculated as follows:
     *
     * (u, v, 1) = K^-1 * (x, y, z)
     */

    void CalculateCameraMatrix(float *x, float *y, float *z)
    {
        *x = (*x) / this->fx + (-this->cx / this->fx) * (*z);
        *y = (*y) / this->fy + (-this->cy / this->fy) * (*z);
        *z = 1.0f;
    }

    void AddOffset(float *x, float *y, float *z)
    {
        *x += this->offset_x;
        *y += this->offset_y;
        *z += this->offset_z;
    }

    void CalcRealPoint(float x_in, float y_in, float z_in, float *x_out, float *y_out, float *z_out)
    {
        float x = x_in;
        float y = y_in;
        float z = z_in;

        this->NormPixel2Camera(&x, &y);

        this->NormPixel2Space(&x, &y, &z);

        this->CalculateCameraMatrix(&x, &y, &z);

        this->AddOffset(&x, &y, &z);

        *x_out = x;
        *y_out = y;
        *z_out = z;
    }

    void CreateIPMMatrix(Mat *matrix)
    {
        float x1, y1, z1;
        float x2, y2, z2;
        float x3, y3, z3;
        float x4, y4, z4;

        this->CalcRealPoint(0.0f, 0.0f, 0.0f, &x1, &y1, &z1);
        this->CalcRealPoint(0.0f, 800.0f, 0.0f, &x2, &y2, &z2);
        this->CalcRealPoint(800.0f, 0.0f, 0.0f, &x3, &y3, &z3);
        this->CalcRealPoint(800.0f, 800.0f, 0.0f, &x4, &y4, &z4);

        float data[16] = {
            x1, y1, z1, 1.0f,
            x2, y2, z2, 1.0f,
            x3, y3, z3, 1.0f,
            x4, y4, z4, 1.0f};

        *matrix = Mat(4, 4, CV_32FC1, data);
    }

private:
    float fx = 476.703f;
    float fy = 476.703f;
    float cx = 400.5f;
    float cy = 400.5f;
    float offset_x = 0.75;
    float offset_y = 0.0;
    float offset_z = 2.025;
};