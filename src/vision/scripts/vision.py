#!/usr/bin/env python3

import rospy
import rospkg
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
import torch
import sys
import os


bridge = CvBridge()

model = None

i = 0


def callback(image_msg):
    global image_mat
    global i
    try:
        image_mat = bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")

        # export_dir = os.path.expanduser("~/export")
        # if not os.path.exists(export_dir):
        #     os.makedirs(export_dir)
        # if i % 20 == 0:
        #     cv.imwrite(os.path.join(export_dir, "frame%d.jpg" % i), image_mat)

        #     print("frame saved at ~/export/frame%d.jpg" % i)
        # i+=1

        model_results = model(image_mat)

        # Display results using opencv
        results = model_results.pandas().xyxy[0]
        # find the closest sign
        closest_sign = None
        closest_sign_dis = 1000000
        closest_x_1 = 0
        closest_x_2 = 0
        closest_y_1 = 0
        closest_y_2 = 0
        for index, row in results.iterrows():
            x1 = int(row['xmin'])
            y1 = int(row['ymin'])
            x2 = int(row['xmax'])
            y2 = int(row['ymax'])
            dis = (x1+x2)/2
            if dis < closest_sign_dis:
                closest_sign_dis = dis
                closest_sign = row['name']
                closest_x_1 = x1
                closest_x_2 = x2
                closest_y_1 = y1
                closest_y_2 = y2

        area_of_boundary_box = (closest_x_2 - closest_x_1) * \
            (closest_y_2 - closest_y_1)

        cv.rectangle(image_mat, (closest_x_1, closest_y_1),
                     (closest_x_2, closest_y_2), (0, 0, 255), 2)
        cv.putText(image_mat, closest_sign, (closest_x_1, closest_y_1),
                   cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv.putText(image_mat, str(area_of_boundary_box), (closest_x_1,
                   closest_y_1 + 30), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        print(closest_sign)

        if area_of_boundary_box > 3500:
            msg_pub_sign = String()
            msg_pub_sign.data = closest_sign
            pub_road_sing_cmd.publish(msg_pub_sign)
        else:
            msg_pub_sign = String()
            msg_pub_sign.data = "none"
            pub_road_sing_cmd.publish(msg_pub_sign)

        cv.imshow("Image window", image_mat)
        cv.waitKey(1)

    except Exception as e:
        rospy.logerr(e)


def init_vision():
    global model_path
    global model

    # get current ros package path
    package_path = rospkg.RosPack().get_path('vision')
    sys.path.append(package_path)  # Add the package directory to sys.path

    model_name = 'best_sign.pt'
    model_path = package_path + '/' + model_name
    print(model_path)

    model = load_model(model_path)


def load_model(path):
    model = model = torch.hub.load('ultralytics/yolov5', 'custom', path=path)
    return model


if _name_ == '_main_':
    try:
        rospy.init_node('vision')
        init_vision()
        sub_image = rospy.Subscriber(
            '/catvehicle/camera_front/image_raw_front', Image, callback)
        pub_road_sing_cmd = rospy.Publisher(
            '/catvehicle/road_sign_cmd', String, queue_size=10)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
