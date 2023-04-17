#!/usr/bin/env python3

import rospy
from vision_msgs.msg import Detection2D
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np
import os
import rospkg
import yaml

import time

import sys
sys.path.append('/home/fgiot-m/ros_catkin_ws/src/yolov7/scripts')



class ObjectDetection:
    yolo = None
    config = None
    bridge = None

    def __init__(self):

        rospy.init_node('yolov7_object_detection_node')
        self.time = 0
        self.count = 0

        rospack = rospkg.RosPack()
        path = rospack.get_path('yolov7')
        self.img_pub_flag = True

        # Load config
        with open(path + "/config/yolov7_object_detection.yaml") as f:
            self.config = yaml.full_load(f)

        weight_path = self.config['classification']['weights']
        if weight_path.endswith('.pt'):
            from pt_detect import DETECT
            self.detect = DETECT(path+weight_path
                                ,self.img_pub_flag)
            print('\nLoad weights file : ',weight_path)
            print('\nLoad pt model...')

        elif weight_path.endswith('.trt'):
            from trt_detect import DETECT
            self.detect = DETECT(path+weight_path
                                ,self.config['classification']['names']
                                ,self.img_pub_flag)
            print('\nLoad weights file : ', weight_path)
            print('\nLoad trt model...')

        else:
            print('error : Please write weights path in yaml file')
            exit()

        self.bridge = CvBridge()

        #change the camera topic if it is different
        rospy.Subscriber('/video_source/raw', Image, self.classify)
        #rospy.Subscriber('/drone_data/image',Image, self.classify)
        rospy.spin()

    def classify(self, image):
        img = self.bridge.imgmsg_to_cv2(image, "rgb8")

        output = self.detect.detect_image(img)
        self.detect.publisher(img, self.bridge, output)


if __name__ == '__main__':
    try:
        detector = ObjectDetection()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start object detection node.')
