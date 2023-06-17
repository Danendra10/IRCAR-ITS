/**
 * To be honest, i dont even fucking know what this code does
 * i got it from github but i forgot who's code is this... i just modify the code a bit
 */
#include "opencv2/opencv.hpp"
#include <vector>

using namespace cv;
using namespace std;

#define CV_FILLED -1

class LaneDetect
{
public:
    Mat currFrame; // stores the upcoming frame
    Mat temp;      // stores intermediate results
    Mat temp2;     // stores the final lane segments
    int diff, diffL, diffR;
    int laneWidth;
    int diffThreshTop;
    int diffThreshLow;
    int ROIrows;
    int vertical_left;
    int vertical_right;
    int vertical_top;
    int smallLaneArea;
    int longLane;
    int vanishingPt;
    float maxLaneWidth;

    // to store various blob properties
    Mat binary_image; // used for blob removal
    int minSize;
    int ratio;
    float contour_area;
    float blob_angle_deg;
    float bounding_width;
    float bounding_length;
    Size2f sz;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    RotatedRect rotated_rect;

    vector<Point> lanes;
    vector<Point> left_lane;
    vector<Point> right_lane;
    vector<Point> middle_lane;

    LaneDetect(Mat startFrame)
    {
        // currFrame = startFrame;                                    //if image has to be processed at original size

        currFrame = Mat(800, 800, CV_8UC1, 0.0);         // initialised the image size to 320x480
        resize(startFrame, currFrame, currFrame.size()); // resize the input to required size

        temp = Mat(currFrame.rows, currFrame.cols, CV_8UC1, 0.0);  // stores possible lane markings
        temp2 = Mat(currFrame.rows, currFrame.cols, CV_8UC1, 0.0); // stores finally selected lane marks

        vanishingPt = currFrame.rows / 2;                      // for simplicity right now
        ROIrows = currFrame.rows - vanishingPt;                // rows in region of interest
        minSize = 0.00015 * (currFrame.cols * currFrame.rows); // min size of any region to be selected as lane
        maxLaneWidth = 0.025 * currFrame.cols;                 // approximate max lane width based on image size
        smallLaneArea = 7 * minSize;
        longLane = 0.3 * currFrame.rows;
        ratio = 4;

        // these mark the possible ROI for vertical lane segments and to filter vehicle glare
        vertical_left = 2 * currFrame.cols / 5;
        vertical_right = 3 * currFrame.cols / 5;
        vertical_top = 2 * currFrame.rows / 3;

        // namedWindow("lane", 2);
        // namedWindow("midstep", 2);
        // namedWindow("currframe", 2);
        // namedWindow("laneBlobs", 2);

        getLane();
    }

    void updateSensitivity()
    {
        int total = 0, average = 0;
        for (int i = vanishingPt; i < currFrame.rows; i++)
            for (int j = 0; j < currFrame.cols; j++)
                total += currFrame.at<uchar>(i, j);
        average = total / (ROIrows * currFrame.cols);
        cout << "average : " << average << endl;
    }

    void getLane()
    {
        for (int i = 0; i < currFrame.rows; i++)
            for (int j = 0; j < currFrame.cols; j++)
            {
                temp.at<uchar>(i, j) = 0;
                temp2.at<uchar>(i, j) = 0;
            }

        // imshow("currframe", currFrame);
        blobRemoval();
    }

    void markLane()
    {
        for (int i = 0; i < currFrame.rows; i++)
        {
            // IF COLOUR IMAGE IS GIVEN then additional check can be done
            //  lane markings RGB values will be nearly same to each other(i.e without any hue)

            // min lane width is taken to be 5
            laneWidth = 5 + maxLaneWidth * (i - 0) / ROIrows;
            for (int j = laneWidth; j < currFrame.cols - laneWidth; j++)
            {

                diffL = currFrame.at<uchar>(i, j) - currFrame.at<uchar>(i, j - laneWidth);
                diffR = currFrame.at<uchar>(i, j) - currFrame.at<uchar>(i, j + laneWidth);
                diff = diffL + diffR - abs(diffL - diffR);

                // 1 right bit shifts to make it 0.5 times
                diffThreshLow = currFrame.at<uchar>(i, j) >> 1;
                // diffThreshTop = 1.2*currFrame.at<uchar>(i,j);

                // both left and right differences can be made to contribute
                // at least by certain threshold (which is >0 right now)
                // total minimum Diff should be atleast more than 5 to avoid noise
                if (diffL > 0 && diffR > 0 && diff > 5)
                    if (diff >= diffThreshLow /*&& diff<= diffThreshTop*/)
                        temp.at<uchar>(i, j) = 255;
            }
        }
    }

    void blobRemoval()
    {
        markLane();

        // find all contours in the binary image
        temp.copyTo(binary_image);
        findContours(binary_image, contours,
                     hierarchy, CV_RETR_CCOMP,
                     CV_CHAIN_APPROX_SIMPLE);

        // for removing invalid blobs
        if (!contours.empty())
        {
            for (size_t i = 0; i < contours.size(); ++i)
            {
                //====conditions for removing contours====//

                contour_area = contourArea(contours[i]);

                // blob size should not be less than lower threshold
                if (contour_area > minSize)
                {
                    rotated_rect = minAreaRect(contours[i]);
                    sz = rotated_rect.size;
                    bounding_width = sz.width;
                    bounding_length = sz.height;

                    // openCV selects length and width based on their orientation
                    // so angle needs to be adjusted accordingly
                    blob_angle_deg = rotated_rect.angle;
                    if (bounding_width < bounding_length)
                        blob_angle_deg = 90 + blob_angle_deg;

                    // if such big line has been detected then it has to be a (curved or a normal)lane
                    if (bounding_length > longLane || bounding_width > longLane)
                    {
                        drawContours(currFrame, contours, i, Scalar(255), CV_FILLED, 8);
                        drawContours(temp2, contours, i, Scalar(255), CV_FILLED, 8);
                    }

                    // angle of orientation of blob should not be near horizontal or vertical
                    // vertical blobs are allowed only near center-bottom region, where centre lane mark is present
                    // length:width >= ratio for valid line segments
                    // if area is very small then ratio limits are compensated
                    else if ((blob_angle_deg < -10 || blob_angle_deg > -10) &&
                             ((blob_angle_deg > -70 && blob_angle_deg < 70) ||
                              (rotated_rect.center.y > vertical_top &&
                               rotated_rect.center.x > vertical_left && rotated_rect.center.x < vertical_right)))
                    {

                        if ((bounding_length / bounding_width) >= ratio || (bounding_width / bounding_length) >= ratio || (contour_area < smallLaneArea && ((contour_area / (bounding_width * bounding_length)) > .75) && ((bounding_length / bounding_width) >= 2 || (bounding_width / bounding_length) >= 2)))
                        {
                            drawContours(currFrame, contours, i, Scalar(255), CV_FILLED, 8);
                            drawContours(temp2, contours, i, Scalar(255), CV_FILLED, 8);
                        }
                    }
                }
            }
        }
        // imshow("midstep", temp);
        // imshow("laneBlobs", temp2);
        // imshow("lane", currFrame);
    }

    void nextFrame(Mat &nxt)
    {
        // currFrame = nxt;                        //if processing is to be done at original size

        resize(nxt, currFrame, currFrame.size()); // resizing the input image for faster processing
        getLane();
    }

    Mat getResult()
    {
        return temp2;
    }

    vector<Point> getLanes()
    {
        vector<Point> lanePoints;
        for (int i = 0; i < currFrame.rows; i++)
            for (int j = 0; j < currFrame.cols; j++)
            {
                if (i > (currFrame.rows - 50))
                    continue;
                if (temp2.at<uchar>(i, j) >= 220 && temp2.at<uchar>(i, j) <= 255)
                    lanePoints.push_back(Point(j, i));
            }
        this->lanes = lanePoints;
        return lanePoints;
    }

    vector<Point> getLeftLane()
    {
        // detect from left bottom to vanishing point, the first detected lane is the left lane
        vector<Point> leftLane;
        for (int i = 0; i < temp2.rows; i++)
            for (int j = 0; j < temp2.cols; j++)
            {
                if (i > (currFrame.rows - 100))
                    continue;
                if (temp2.at<uchar>(i, j) >= 220 && temp2.at<uchar>(i, j) <= 255)
                {
                    leftLane.push_back(Point(j, i));
                    if (abs(leftLane.back().x - j) > 100)
                        break;
                    break;
                }
                if (j >= temp2.cols * 2 / 3)
                    break;
            }
        this->left_lane = leftLane;
        return leftLane;
    }

    vector<Point> getRightLane()
    {
        vector<Point> rightLane;
        for (int i = 0; i < temp2.rows; i++)
            for (int j = temp2.cols; j > 0; j--)
            {
                if (i > (currFrame.rows - 100))
                    continue;
                if (temp2.at<uchar>(i, j) >= 220 && temp2.at<uchar>(i, j) <= 255)
                {
                    rightLane.push_back(Point(j, i));
                    if (abs(rightLane.back().x - j) > 100)
                        break;
                    break;
                }
                if (j <= temp2.cols / 2)
                    break;
            }

        this->right_lane = rightLane;
        return rightLane;
    }

    vector<Point> calcMiddleLane()
    {
        vector<Point> left = this->left_lane;
        vector<Point> right = this->right_lane;
        vector<Point> middle;
        int calc_buffer_x_left, calc_buffer_y_left;
        int calc_buffer_x_right, calc_buffer_y_right;

        for (int i = 0; i < left.size() && i < right.size(); i++)
        {
            if (left[i].x != 0 && left[i].y != 0)
            {
                calc_buffer_x_left = left[i].x;
                calc_buffer_y_left = left[i].y;
            }
            else
            {
                left[i].x = calc_buffer_x_left;
                left[i].y = calc_buffer_y_left;
            }

            if (right[i].x != 0 && right[i].y != 0)
            {
                calc_buffer_x_right = right[i].x;
                calc_buffer_y_right = right[i].y;
            }
            else
            {
                right[i].x = calc_buffer_x_right;
                right[i].y = calc_buffer_y_right;
            }

            Point mid = Point((left[i].x + right[i].x) / 2, (left[i].y + right[i].y) / 2);
            middle.push_back(mid);
            this->middle_lane.push_back(mid);
        }
        return middle;
    }
};
