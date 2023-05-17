import cv2 as cv
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from datetime import datetime, timedelta
from std_msgs.msg import UInt8

bridge = CvBridge()


def click_event(event, x, y, flags, param):
    global raw_frame
    if event == cv.EVENT_LBUTTONDOWN:
        print(x, ', ', y)
        font = cv.FONT_HERSHEY_SIMPLEX
        strXY = str(x) + ', ' + str(y)
        cv.putText(raw_frame, strXY, (x, y), font, 1, (255, 255, 0), 2)
        cv.imshow('frame_raw', raw_frame)


def DetectLane(image):
    # Convert image to grayscale
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

    # Apply Gaussian blur
    blur = cv.GaussianBlur(gray, (5, 5), 0)

    # Perform Canny edge detection
    edges = cv.Canny(blur, 50, 150)

    # Define a region of interest (ROI) polygon
    height, width = edges.shape
    roi_vertices = np.array(
        [[(0, height), (0, height), (width, height), (width, height)]], dtype=np.int32)
    mask = np.zeros_like(edges)
    cv.fillPoly(mask, roi_vertices, 255)

    # Apply the mask to extract the ROI
    masked_edges = cv.bitwise_and(edges, mask)

    # Apply Hough transform to detect lines
    lines = cv.HoughLinesP(masked_edges, rho=1, theta=np.pi /
                           180, threshold=50, minLineLength=50, maxLineGap=100)

    # Separate lines into left and right lanes based on their slopes
    left_lane_slope = []
    right_lane_slope = []
    left_lane_intercept = []
    right_lane_intercept = []
    # print(f"{lines}\n\n")

    # sorted_lines = sorted()

    for line in lines:
        x1, y1, x2, y2 = line
        slope = (y2 - y1) / (x2 - x1)
        intercept = y1 - slope * x1

        if slope > 0:
            left_lane_slope.append(slope)
            left_lane_intercept.append(intercept)
        else:
            right_lane_slope.append(slope)
            right_lane_intercept.append(intercept)
    # print(f"left {left_lane_intercept} || right {right_lane_intercept}\n\n")

    # Calculate average slope and intercept for left lane
    left_slope = np.mean(left_lane_slope)
    left_intercept = np.mean(left_lane_intercept)
    left_lane = ((height - left_intercept) / left_slope,
                 0), ((height / 2 - left_intercept) / left_slope, height / 2)

    # Calculate average slope and intercept for right lane
    right_slope = np.mean(right_lane_slope)
    right_intercept = np.mean(right_lane_intercept)
    right_lane = ((height - right_intercept) / right_slope,
                  0), ((height / 2 - right_intercept) / right_slope, height / 2)

    return left_lane, right_lane, masked_edges, mask


def detect(image):
    # Convert image to grayscale
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

    # Apply Gaussian blur
    blur = cv.GaussianBlur(gray, (5, 5), 0)

    # Perform Canny edge detection
    edges = cv.Canny(blur, 50, 150)

    # Define a region of interest (ROI) polygon
    height, width = edges.shape
    roi_vertices = np.array(
        [[(0, height), (0, height * 1 / 5), (width, height * 1 / 5), (width, height)]], dtype=np.int32)
    mask = np.zeros_like(edges)
    cv.fillPoly(mask, roi_vertices, 255)

    # Apply the mask to extract the ROI
    masked_edges = cv.bitwise_and(edges, mask)

    # Apply Hough transform to detect lines
    lines = cv.HoughLinesP(masked_edges, rho=1, theta=np.pi /
                           180, threshold=50, minLineLength=50, maxLineGap=100)
    np_lines = np.array(lines)

    # Extract the values from the array
    x1 = np_lines[:, 0, 0]
    y1 = np_lines[:, 0, 1]
    x2 = np_lines[:, 0, 2]
    y2 = np_lines[:, 0, 3]

    # Sort the array based on the first index (arr[0][0])
    sorted_indices = np.argsort(x1)
    sorted_arr = np_lines[sorted_indices]

    # Sort the values based on the sorted indices
    x1_sorted = x1[sorted_indices]
    y1_sorted = y1[sorted_indices]
    x2_sorted = x2[sorted_indices]
    y2_sorted = y2[sorted_indices]
    for i in range(sorted_arr.shape[0]):
        x1 = x1_sorted[i]
        x2 = x2_sorted[i]
        y1 = y1_sorted[i]
        y2 = y2_sorted[i]
        cv.line(raw_frame, (x1, y1), (x2, y2), (0, 0, 255), 2)

    line_pts = []
    # if the color in the masked_edges is white, then it is a line and store it to arra
    for i in range(len(masked_edges)):
        for j in range(len(masked_edges[i])):
            if (masked_edges[i][j] == 255):
                line_pts.append((j, i))

    return lines, masked_edges, x1_sorted, y1_sorted, x2_sorted, y2_sorted, line_pts


def average_slope_intercept(lines):
    """
    Find the slope and intercept of the left and right lanes of each image.
        Parameters:
            lines: The output lines from Hough Transform.
    """
    left_lines = []  # (slope, intercept)
    left_weights = []  # (length,)
    right_lines = []  # (slope, intercept)
    right_weights = []  # (length,)

    for line in lines:
        for x1, y1, x2, y2 in line:
            if x1 == x2:
                continue
            slope = (y2 - y1) / (x2 - x1)
            intercept = y1 - (slope * x1)
            length = np.sqrt(((y2 - y1) ** 2) + ((x2 - x1) ** 2))
            if slope < 0:
                left_lines.append((slope, intercept))
                left_weights.append((length))
            else:
                right_lines.append((slope, intercept))
                right_weights.append((length))

    # add more weight to longer lines
    left_lane = np.dot(left_weights, left_lines) / \
        np.sum(left_weights) if len(left_weights) > 0 else None
    right_lane = np.dot(right_weights, right_lines) / \
        np.sum(right_weights) if len(right_weights) > 0 else None

    return left_lane, right_lane


def average_slope_intercept(lines):
    """
    Find the slope and intercept of the left and right lanes of each image.
        Parameters:
            lines: The output lines from Hough Transform.
    """
    left_lines = []  # (slope, intercept)
    left_weights = []  # (length,)
    right_lines = []  # (slope, intercept)
    right_weights = []  # (length,)

    for line in lines:
        for x1, y1, x2, y2 in line:
            if x1 == x2:
                continue
            slope = (y2 - y1) / (x2 - x1)
            intercept = y1 - (slope * x1)
            length = np.sqrt(((y2 - y1) ** 2) + ((x2 - x1) ** 2))
            if slope < 0:
                left_lines.append((slope, intercept))
                left_weights.append((length))
            else:
                right_lines.append((slope, intercept))
                right_weights.append((length))
    left_lane = np.dot(left_weights,  left_lines) / \
        np.sum(left_weights) if len(left_weights) > 0 else None
    right_lane = np.dot(right_weights, right_lines) / \
        np.sum(right_weights) if len(right_weights) > 0 else None
    return left_lane, right_lane


def pixel_points(y1, y2, line):
    """
    Converts the slope and intercept of each line into pixel points.
        Parameters:
            y1: y-value of the line's starting point.
            y2: y-value of the line's end point.
            line: The slope and intercept of the line.
    """
    if line is None:
        return None
    slope, intercept = line
    x1 = int((y1 - intercept)/slope)
    x2 = int((y2 - intercept)/slope)
    y1 = int(y1)
    y2 = int(y2)
    return ((x1, y1), (x2, y2))


def lane_lines(image, lines):
    """
    Create full lenght lines from pixel points.
        Parameters:
            image: The input test image.
            lines: The output lines from Hough Transform.
    """
    left_lane, right_lane = average_slope_intercept(lines)
    y1 = image.shape[0]
    y2 = y1 * 0.6
    left_line = pixel_points(y1, y2, left_lane)
    right_line = pixel_points(y1, y2, right_lane)
    return left_line, right_line


def draw_lane_lines(image, lines, color=[255, 0, 0], thickness=12):
    """
    Draw lines onto the input image.
        Parameters:
            image: The input test image.
            lines: The output lines from Hough Transform.
            color (Default = red): Line color.
            thickness (Default = 12): Line thickness. 
    """
    line_image = np.zeros_like(image)
    for line in lines:
        if line is not None:
            cv.line(line_image, *line,  color, thickness)
    return cv.addWeighted(image, 1.0, line_image, 1.0, 0.0)


def DetermineCurrPose(x1, y1, x2, y2):
    global now
    state = 0
    cnt_total_in_left = 0
    cnt_total_in_right = 0
    cnt_total_in_middle = 0
    for i in range(len(x1)):
        if (x1[i] <= (400 - 100)):
            cnt_total_in_left += 1
        elif (x1[i] >= (400+100)):
            cnt_total_in_right += 1
        else:
            cnt_total_in_middle += 1

    # print("Current Time =", now)
    # print("current_time diff = ", datetime.now() - now)
    delta = datetime.now() - now
    delta = (str)(delta)
    delta = (int)(delta[6])
    if (delta > 2):
        state_right = (int)(
            (cnt_total_in_right > cnt_total_in_middle and cnt_total_in_right > cnt_total_in_left))
        state_middle = (int)(
            (cnt_total_in_middle > cnt_total_in_right and cnt_total_in_middle > cnt_total_in_left))
        state_left = (int)(
            (cnt_total_in_left > cnt_total_in_middle and cnt_total_in_left > cnt_total_in_right))
        print(cnt_total_in_right, cnt_total_in_left,
              cnt_total_in_middle, state_left, state_middle, state_right)
        now = datetime.now()
        current_time = now.strftime("%H:%M:%S")
    return state_right, state_middle, state_left


def GetMiddlePtsLeft(left, middle):
    middle_pts_left = []
    for i in range(len(left)):
        sum_x = left[i][0] + middle[i][0]
        sum_y = left[i][1] + middle[i][1]

        avg_x = (int)(sum_x/2)
        avg_y = (int)(sum_y/2)
        middle_pts_left.append((avg_x, avg_y))

    return middle_pts_left


def GetMiddlePtsRight(right, middle):
    middle_pts_right = []
    for i in range(len(right)):
        sum_x = right[i][0] + middle[i][0]
        sum_y = right[i][1] + middle[i][1]

        avg_x = (int)(sum_x/2)
        avg_y = (int)(sum_y/2)
        middle_pts_right.append((avg_x, avg_y))

    return middle_pts_right


def ImageCllbck(frame):
    global raw_frame, new_frame

    raw_frame = bridge.imgmsg_to_cv2(frame, "bgr8")

    pt_a = (2,  414)
    pt_b = (2,  670)
    pt_c = (798,  670)
    pt_d = (798,  414)

    width_AD = np.sqrt((pt_a[0] - pt_d[0])**2 + (pt_a[1] - pt_d[1])**2)
    width_BC = np.sqrt((pt_b[0] - pt_c[0])**2 + (pt_b[1] - pt_c[1])**2)

    height_AB = np.sqrt((pt_a[0] - pt_b[0])**2 + (pt_a[1] - pt_b[1])**2)
    height_CD = np.sqrt((pt_c[0] - pt_d[0])**2 + (pt_c[1] - pt_d[1])**2)

    max_width = max(int(width_AD), int(width_BC))
    max_height = max(int(height_AB), int(height_CD))

    input_pts = np.float32([pt_a, pt_b, pt_c, pt_d])

    output_pts = np.float32(
        [[0, 0], [0, 800], [800, 800], [800, 0]])

    Mask = cv.getPerspectiveTransform(input_pts, output_pts)

    wraped_frame = cv.warpPerspective(
        raw_frame, Mask, (800, 800), flags=cv.INTER_LINEAR)

    lines, new_frame, x1_arr, y1_arr, x2_arr, y2_arr, line_pts = detect(
        wraped_frame)

    left_line_arr = []
    right_line_arr = []
    middle_line_arr = []

    # iterate from pixel 0, 800
    

    # draw the right and left lines
    # for i in range(len(left_line_arr)):
    #     cv.circle(wraped_frame, left_line_arr[i], 5, (0, 0, 255), -1)
    # for i in range(len(right_line_arr)):
    #     cv.circle(wraped_frame, right_line_arr[i], 5, (0, 255, 0), -1)
    # for i in range(len(middle_line_arr)):
    #     cv.circle(wraped_frame, middle_line_arr[i], 5, (255, 0, 255), -1)

    left_middle_pts = []
    right_middle_pts = []

    # for i in range(len(left_middle_pts)):
    #     cv.circle(wraped_frame, left_middle_pts[i], 5, (255, 0, 0), -1)
    # for i in range(len(right_middle_pts)):
    #     cv.circle(wraped_frame, right_middle_pts[i], 5, (255, 0, 0), -1)
    # for i in range(len(left_middle_pts)):
    #     cv.circle(wraped_frame, left_middle_pts[i], 5, (255, 0, 0), -1)

    # state_right, state_middle, state_left = DetermineCurrPose(
    #     x1_arr, y1_arr, x2_arr, y2_arr)

    # if (state_right != 0 and state_left == 0 and state_middle == 0):
    #     state = 1
    # elif (state_middle != 0 and state_left == 0 and state_right == 0):
    #     state = 2
    # elif (state_left != 0 and state_right == 0 and state_middle == 0):
    #     state = 3
    # else:
    #     state = 2

    # print(state)

    cv.imshow("masked", new_frame)
    cv.imshow("wraped", wraped_frame)
    cv.setMouseCallback('frame_raw', click_event)
    cv.waitKey(1)


def init():
    global now, current_time
    now = datetime.now()
    current_time = now.strftime("%H:%M:%S")


def main():
    global pub_state
    rospy.init_node('vision', anonymous=True)
    rospy.loginfo("HAI")
    init()
    rospy.Subscriber(
        "/catvehicle/camera_front/image_raw_front", Image, ImageCllbck)
    pub_state = rospy.Publisher("/catvehicle/state", UInt8, queue_size=1)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
