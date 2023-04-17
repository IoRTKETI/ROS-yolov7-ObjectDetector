import cv2
import torch
import time
import argparse
from numpy import random
import yaml

from utils.trt_yolo_plugin import TRT_engine
from utils.detect_uilts import add_camera_args, Detect, open_window

import rospkg
import rospy

from sensor_msgs.msg import Image
from yolov7.msg import Detector2DArray
from yolov7.msg import Detector2D
from yolov7.msg import Bbox2D

from cv_bridge import CvBridge, CvBridgeError



class DETECT(object):
    def __init__(self, weights, names, img_b_flag):
        self.model = weights
        self.img_size = 640
        self.device = torch.device('cuda:0')
        self.half = self.device.type != 'cpu'  # half precision only supported on CUDA
        self.img_b_pub = img_b_flag

        self.names = names
        self.class_color = [[random.randint(0, 255) for _ in range(3)] for _ in self.names]

        # setup pub node
        self.bbox_pub = rospy.Publisher("/robot_data/bbox", Detector2DArray, queue_size=1)
        if self.img_b_pub is True:
            self.img_pub = rospy.Publisher("/robot_data/raw_bbox", Image, queue_size=10)

        # setup trt_predict engine
        self.trt_yolo = TRT_engine(self.model)


    def detect_image(self, img):
        # pred: x1, y1, x2, y2, conf, cls_pred
        pred = self.trt_yolo.predict(img, threshold=0.5)

        return pred


    def draw_bbox_image(self, img, pred):
        # Bounding-box colors

        for i in range(len(pred)):
            # pred: x1, y1, x2, y2, conf, cls_pred
            text = f"{self.names[int(pred[i][5])]}:{pred[i][4]:0.2f}"

            img = cv2.rectangle(img, (int(pred[i][0]), int(pred[i][1])), (int(pred[i][2]), int(pred[i][3])), self.class_color[int(pred[i][5])], 2)
            img = cv2.line(img, (int((pred[i][0]+pred[i][2])/2), int((pred[i][1]+pred[i][3])/2)), (int((pred[i][0]+pred[i][2])/2), int((pred[i][1]+pred[i][3])/2)), self.class_color[int(pred[i][5])], 4)
            font =  cv2.FONT_HERSHEY_PLAIN
            img = cv2.putText(img, text, (int(pred[i][0]), int(pred[i][1])-5), font, 2, self.class_color[int(pred[i][5])], 1, cv2.LINE_AA)
        return img


    def publisher(self, img, bridge, pred):
        """Publish ROS messages."""

        try:
            # Publish bbox
            detection_ary = Detector2DArray()
            detection_ary.header.stamp = rospy.Time.now()


            for i in range(len(pred)):
                # pred: x1, y1, x2, y2, conf, cls_pred
                detection = Detector2D()
                detection.header.stamp = rospy.Time.now()

                cls_id = int(pred[i][5])
                #self.count_class_num(cls_id)

                detection.id = cls_id
                detection.confidence_score = float(pred[i][4])

                detection.bbox.header.stamp = rospy.Time.now()
                detection.bbox.center_x = int(pred[i][0] + (pred[i][2] - pred[i][0])/2)
                detection.bbox.center_y = int(pred[i][1] + (pred[i][3] - pred[i][1])/2)

                detection.bbox.size_x = int(abs(pred[i][0] - pred[i][2]))
                detection.bbox.size_y = int(abs(pred[i][1] - pred[i][3]))

                detection_ary.detections.append(detection)


            # Publish all
            if self.img_b_pub is True:
                img_out = self.draw_bbox_image(img, pred)

                img_msg = bridge.cv2_to_imgmsg(img_out, "rgb8")
                img_msg.header.stamp = rospy.Time.now()

                self.img_pub.publish(img_msg)
            self.bbox_pub.publish(detection_ary)


        except CvBridgeError as err:
            print(err)

        return
