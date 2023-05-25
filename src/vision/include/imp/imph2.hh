#ifndef __IMP_HH_
#define __IMP_HH_

#include <iostream>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;

#include <math.h>

#define CAMERA_POS_X 0
#define CAMERA_POS_Y 0
#define CAMERA_POS_Z 60
#define FOV_H 50.0f
#define FOV_V 50.0f
#define DEG2RAD 0.01745329251994329576923690768489f

#define SRC_RESIZED_WIDTH 800
#define SRC_RESIZED_HEIGHT 800
#define DST_REMAPPED_WIDTH 800
#define DST_REMAPPED_HEIGHT 800

int vanishing_point_x;
int vanishing_point_y;

int *ipm_table;

/**
 * @param srcw width of the source image
 * @param srch height of the source image
 * @param dstw width of the destination image
 * @param dsth height of the destination image
 * @param vptx x coordinate of the vanishing point
 * @param vpty y coordinate of the vanishing point
 * @param maptable pointer to the map table
 *
 * @return void
 *
 * @brief Build a map table for the inverse perspective mapping
 */
void BuildIPMTable(int srcw, int srch, int dstw, int dsth, int vptx, int vpty, int *maptable)
{
    /**
     * @brief Calculate the angles of the camera
     */
    const float alpha_h = 0.5f * FOV_H * DEG2RAD;
    const float alpha_v = 0.5f * FOV_V * DEG2RAD;

    /**
     * @brief Calculate the angles of the camera
     */
    const float gamma = -(float)(vptx - (srcw >> 1)) * alpha_h / (srcw >> 1); // camera pan angle
    const float theta = -(float)(vpty - (srch >> 1)) * alpha_v / (srch >> 1); // camera tilt angle

    /**
     * @brief Calculate the height of the camera
     */
    const int front_map_start_position = dsth >> 1;
    const int front_map_end_position = front_map_start_position + dsth;
    const int side_map_mid_position = dstw >> 1;

    const int front_map_scale_factor = 4;
    const int side_map_scale_factor = 2;

    /**
     * @brief Loop through the destination image
     */
    for (int y = 0; y < dstw; ++y)
    {
        for (int x = front_map_start_position; x < front_map_end_position; ++x)
        {
            int idx = y * dsth + (x - front_map_start_position);

            int deltax = front_map_scale_factor * (front_map_end_position - x - CAMERA_POS_X);
            int deltay = side_map_scale_factor * (y - side_map_mid_position - CAMERA_POS_Y);

            if (deltay == 0)
            {
                maptable[idx] = maptable[idx - dsth];
            }
            else
            {
                int u = (int)((atan(CAMERA_POS_Z * sin(atan((float)deltay / deltax)) / deltay) - (theta - alpha_v)) / (2 * alpha_v / srch));
                int v = (int)((atan((float)deltay / deltax) - (gamma - alpha_h)) / (2 * alpha_h / srcw));
                if (u >= 0 && u < srch && v >= 0 && v < srcw)
                {
                    maptable[idx] = srcw * u + v;
                }
                else
                {
                    maptable[idx] = -1;
                }
            }
        }
    }
}

void InversePerspectiveMapping(int dstw, int dsth, unsigned char *src, int *maptable, unsigned char *dst)
{
    int idx = 0;
    for (int j = 0; j < dsth; ++j)
    {
        for (int i = 0; i < dstw; ++i)
        {
            if (maptable[idx] != -1)
            {
                dst[i * dsth + j] = src[maptable[idx]];
            }
            else
            {
                dst[i * dsth + j] = 0;
            }
            ++idx;
        }
    }
}

Mat DrawLineEachMeter(Mat &src, float meterInterval)
{
    Mat result = src.clone();

    int width = result.cols;
    int height = result.rows;

    int intervalPixels = static_cast<int>(meterInterval * (DST_REMAPPED_WIDTH / (FOV_H * DEG2RAD)));

    // draw line in middle bottom
    line(result, Point(400, height), Point(width, height - 400), Scalar(0, 0, 255), 2);

    for (int i = intervalPixels; i < width; i += intervalPixels)
    {
        line(result, Point(i, 0), Point(i, height - 1), Scalar(0, 0, 255), 2);
        printf("i: %d\n", i);
    }

    for (int j = intervalPixels; j < height; j += intervalPixels)
    {
        line(result, Point(0, j), Point(width - 1, j), Scalar(0, 0, 255), 2);
    }

    return result;
}

#endif // __IMP_HH_