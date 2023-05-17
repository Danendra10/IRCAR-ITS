import cv2 as cv
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from datetime import datetime, timedelta
from std_msgs.msg import UInt8

bridge = CvBridge()


def HSL_color_selection(image):
    """
    Apply color selection to the HSL images to blackout everything except for white and yellow lane lines.
        Parameters:
            image: An np.array compatible with plt.imshow.
    """
    # Convert the input image to HSL
    converted_image = cv.cvtColor(image, cv.COLOR_RGB2HLS)

    # White color mask
    lower_threshold = np.uint8([0, 200, 0])
    upper_threshold = np.uint8([255, 255, 255])
    white_mask = cv.inRange(converted_image, lower_threshold, upper_threshold)

    # Yellow color mask
    lower_threshold = np.uint8([10, 0, 100])
    upper_threshold = np.uint8([40, 255, 255])
    yellow_mask = cv.inRange(
        converted_image, lower_threshold, upper_threshold)

    # Combine white and yellow masks
    mask = cv.bitwise_or(white_mask, yellow_mask)
    masked_image = cv.bitwise_and(image, image, mask=mask)

    return masked_image


def hough_transform(image):
    """
    Determine and cut the region of interest in the input image.
        Parameters:
            image: The output of a Canny transform.
    """
    rho = 1  # Distance resolution of the accumulator in pixels.
    theta = np.pi/180  # Angle resolution of the accumulator in radians.
    # Only lines that are greater than threshold will be returned.
    threshold = 20
    minLineLength = 20  # Line segments shorter than that are rejected.
    maxLineGap = 300  # Maximum allowed gap between points on the same line to link them
    return cv.HoughLinesP(image, rho=rho, theta=theta, threshold=threshold,
                          minLineLength=minLineLength, maxLineGap=maxLineGap)


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


def BirdEye(frame):
    pt_a = (2,  414)
    pt_b = (2,  670)
    pt_c = (798,  670)
    pt_d = (798,  414)

    width_AD = np.sqrt((pt_a[0] - pt_d[0])**2 + (pt_a[1] - pt_d[1])**2)
    width_BC = np.sqrt((pt_b[0] - pt_c[0])**2 + (pt_b[1] - pt_c[1])**2)

    height_AB = np.sqrt((pt_a[0] - pt_b[0])**2 + (pt_a[1] - pt_b[1])**2)
    height_CD = np.sqrt((pt_c[0] - pt_d[0])**2 + (pt_c[1] - pt_d[1])**2)

    input_pts = np.float32([pt_a, pt_b, pt_c, pt_d])

    output_pts = np.float32(
        [[0, 0], [0, 800], [800, 800], [800, 0]])

    Mask = cv.getPerspectiveTransform(input_pts, output_pts)

    wraped_frame = cv.warpPerspective(
        frame, Mask, (800, 800), flags=cv.INTER_LINEAR)

    return wraped_frame


def ImageCllbck(frame):
    global raw_frame, new_frame

    raw_frame = bridge.imgmsg_to_cv2(frame, "bgr8")

    wrapped = BirdEye(raw_frame)

    cv.imshow("wrapped", wrapped)
    cv.imshow("hsl", hsl)
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
