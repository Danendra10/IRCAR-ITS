#ifndef __IMP_HH_
#define __IMP_HH_

#include <iostream>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <math.h>
#include "entity/entity.hh"
#include "logger/logger.h"

using namespace cv;

CameraParameters cam_params;
int *maptable = new int[800 * 800];

int vanishing_point_x;
int vanishing_point_y;

void LogParams()
{
    Logger(GREEN, "                    Camera parameters\n");
    Logger(GREEN, "==============================================================\n");
    Logger(GREEN, "Horizontal FOV      : %f\n", cam_params.horizontal_fov);
    Logger(GREEN, "Vertical FOV        : %f\n", cam_params.vertical_fov);
    Logger(GREEN, "Image width         : %d\n", cam_params.image_width);
    Logger(GREEN, "Image height        : %d\n", cam_params.image_height);
    Logger(GREEN, "Near clip           : %f\n", cam_params.near_clip);
    Logger(GREEN, "Far clip            : %f\n", cam_params.far_clip);
    Logger(GREEN, "Noise mean          : %f\n", cam_params.noise_mean);
    Logger(GREEN, "Noise std dev       : %f\n", cam_params.noise_std_dev);
    Logger(GREEN, "Hack baseline       : %f\n", cam_params.hack_baseline);
    Logger(GREEN, "Distortion k1       : %f\n", cam_params.distortion_k1);
    Logger(GREEN, "Distortion k2       : %f\n", cam_params.distortion_k2);
    Logger(GREEN, "Distortion k3       : %f\n", cam_params.distortion_k3);
    Logger(GREEN, "Distortion t1       : %f\n", cam_params.distortion_t1);
    Logger(GREEN, "Distortion t2       : %f\n", cam_params.distortion_t2);
    Logger(GREEN, "Camera position x   : %f\n", cam_params.camera_pos_x);
    Logger(GREEN, "Camera position y   : %f\n", cam_params.camera_pos_y);
    Logger(GREEN, "Camera position z   : %f\n", cam_params.camera_pos_z);
    Logger(GREEN, "Camera scale x      : %f\n", cam_params.cam_scale_x);
    Logger(GREEN, "Camera scale y      : %f\n", cam_params.cam_scale_y);
    Logger(GREEN, "==============================================================\n");
}

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

void BuildIPMTable(const int src_w, const int src_h, const int dst_w, const int dst_h, const int vanishing_pt_x, const int vanishing_pt_y, int *maptable)
{
    // float alpha = cam_params.horizontal_fov / 2;
    // float gamma = -(float)(vanishing_pt_x - (src_w >> 1)) * alpha / (src_w >> 1);
    // float theta = -(float)(vanishing_pt_y - (src_h >> 1)) * alpha / (src_h >> 1);

    // frame conversion from source(800x800) to desired frame (800x800) with the calculation of vanishing point(400x400)
    float alpha = 0.686111;
    float gamma = 0;
    float theta = 0.069444;

    int front_map_pose_start = (dst_h >> 1);                     // shifting binary div by 2
    int front_map_pose_end = front_map_pose_start + dst_h + 200; // but how ?? i still dont get it

    int side_map_mid_pose = dst_w >> 1; // looking for the middle of the road

    // int front_map_scale = cam_params.cam_scale_y;
    // int side_map_scale = cam_params.cam_scale_x;

    int front_map_scale = 2; // i'm kinda confused but the bigger the value, road projected more far away
    int side_map_scale = 0;

    for (int y = 0; y < dst_w; ++y)
    {
        for (int x = front_map_pose_start; x < front_map_pose_end; ++x)
        {
            int index = y * dst_h + (x - front_map_pose_start);

            int delta_x = front_map_scale * (front_map_pose_end - x - cam_params.camera_pos_x);
            int delta_y = front_map_scale * (y - side_map_mid_pose - cam_params.camera_pos_y);

            if (!delta_y)
                maptable[index] = maptable[index - dst_h];
            else
            {
                int u = (int)((atan(cam_params.camera_pos_z * sin(atan((float)delta_y / delta_x)) / delta_y) - (theta - alpha)) / (2 * alpha / src_h));
                int v = (int)((atan((float)delta_y / delta_x) - (gamma - alpha)) / (2 * alpha / src_w));

                if (u >= 0 && u < src_h && v >= 0 && v < src_w)
                    maptable[index] = src_w * u + v;
                else
                    maptable[index] = -1;
            }
        }
    }
}

void InversePerspective(const int dst_w, const int dst_h, const unsigned char *src, const int *maptable, unsigned char *dst)
{
    int index = 0;
    for (int j = 0; j < dst_h; ++j)
    {
        for (int i = 0; i < dst_w; ++i)
        {
            if (maptable[index] == -1)
            {
                dst[i * dst_h + j] = 0;
                // ROS_ERROR("%d | %c", i * dst_h + j, dst[i * dst_h + j]);
            }
            else
            {
                dst[i * dst_h + j] = src[maptable[index]];
                // ROS_WARN("%d | %c", i * dst_h + j, dst[i * dst_h + j]);
            }
            ++index;
        }
    }
}

void MaptablePxToM(int *maptable, int maptable_size, int *maptable_m)
{
    for (int i = 0; i < maptable_size; ++i)
    {
        maptable_m[i] = (int)(maptable[i] * cam_params.cam_scale_x / 100);
    }
}

float PxToM(int p)
{
    return (float)(p * cam_params.cam_scale_x / 100);
}

float MToPx(int p)
{
    return (float)(p * 100 / cam_params.cam_scale_x);
}
// void BuildIPMTable(int srcw, int srch, int dstw, int dsth, int vptx, int vpty, int *maptable)
// {
//     /**
//      * @brief Calculate the angles of the camera
//      */
//     const float alpha_h = 0.5f * FOV_H * DEG2RAD;
//     const float alpha_v = 0.5f * FOV_V * DEG2RAD;

//     /**
//      * @brief Calculate the angles of the camera
//      */
//     const float gamma = -(float)(vptx - (srcw >> 1)) * alpha_h / (srcw >> 1); // camera pan angle
//     const float theta = -(float)(vpty - (srch >> 1)) * alpha_v / (srch >> 1); // camera tilt angle

//     /**
//      * @brief Calculate the height of the camera
//      */
//     const int front_map_start_position = dsth >> 1;
//     const int front_map_end_position = front_map_start_position + dsth;
//     const int side_map_mid_position = dstw >> 1;

//     const int front_map_scale_factor = 4;
//     const int side_map_scale_factor = 2;

//     /**
//      * @brief Loop through the destination image
//      */
//     for (int y = 0; y < dstw; ++y)
//     {
//         for (int x = front_map_start_position; x < front_map_end_position; ++x)
//         {
//             int idx = y * dsth + (x - front_map_start_position);

//             int deltax = front_map_scale_factor * (front_map_end_position - x - CAMERA_POS_X);
//             int deltay = side_map_scale_factor * (y - side_map_mid_position - CAMERA_POS_Y);

//             if (deltay == 0)
//             {
//                 maptable[idx] = maptable[idx - dsth];
//             }
//             else
//             {
//                 int u = (int)((atan(CAMERA_POS_Z * sin(atan((float)deltay / deltax)) / deltay) - (theta - alpha_v)) / (2 * alpha_v / srch));
//                 int v = (int)((atan((float)deltay / deltax) - (gamma - alpha_h)) / (2 * alpha_h / srcw));
//                 if (u >= 0 && u < srch && v >= 0 && v < srcw)
//                 {
//                     maptable[idx] = srcw * u + v;
//                 }
//                 else
//                 {
//                     maptable[idx] = -1;
//                 }
//             }
//         }
//     }
// }

// void InversePerspectiveMapping(int dstw, int dsth, unsigned char *src, int *maptable, unsigned char *dst)
// {
//     int idx = 0;
//     for (int j = 0; j < dsth; ++j)
//     {
//         for (int i = 0; i < dstw; ++i)
//         {
//             if (maptable[idx] != -1)
//             {
//                 dst[i * dsth + j] = src[maptable[idx]];
//             }
//             else
//             {
//                 dst[i * dsth + j] = 0;
//             }
//             ++idx;
//         }
//     }
// }

// Mat DrawLineEachMeter(Mat &src, float meterInterval)
// {
//     Mat result = src.clone();

//     int width = result.cols;
//     int height = result.rows;

//     int intervalPixels = static_cast<int>(meterInterval * (DST_REMAPPED_WIDTH / (FOV_H * DEG2RAD)));

//     // draw line in middle bottom
//     line(result, Point(400, height), Point(width, height - 400), Scalar(0, 0, 255), 2);

//     for (int i = intervalPixels; i < width; i += intervalPixels)
//     {
//         line(result, Point(i, 0), Point(i, height - 1), Scalar(0, 0, 255), 2);
//         printf("i: %d\n", i);
//     }

//     for (int j = intervalPixels; j < height; j += intervalPixels)
//     {
//         line(result, Point(0, j), Point(width - 1, j), Scalar(0, 0, 255), 2);
//     }

//     return result;
// }

#endif // __IMP_HH_