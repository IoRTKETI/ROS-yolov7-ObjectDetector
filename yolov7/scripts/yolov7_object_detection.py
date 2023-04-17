#!/usr/bin/env python3

import rospy
import rospkg
import yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import sys
sys.path.append('/home/fgiot-m/ros_catkin_ws/src/yolov7/scripts')

from pt_detect import DETECT as PtDETECT
from trt_detect import DETECT as TrtDETECT


class ObjectDetection:
    def __init__(self):
        rospy.init_node('yolov7_object_detection_node')

        rospack = rospkg.RosPack()
        path = rospack.get_path('yolov7')
        self.yaml_file = 'yolov7_object_detection.yaml'
        self.path_yaml_file = f"{path}/config/{self.yaml_file}"

        # change False if you don't want publish detected image (only want bbox topic) 
        self.img_pub_flag = True

        # change the subscribed topic if it is different
        subscribing_image_topic = '/video_source/raw'
        #subscribing_image_topic = '/drone_data/image'


        # Load config
        with open(self.path_yaml_file) as f:
            self.config = yaml.full_load(f)

        weight_path = self.config['classification']['weights']
        if weight_path.endswith('.pt'):
            print('\nLoading pt model...')
 
            self.detect = PtDETECT(f"{path}/{weight_path}"
                                ,self.img_pub_flag)

        elif weight_path.endswith('.trt'):
            print('\nLoading trt model...')
            self.detect = TrtDETECT(f"{path}/{weight_path}"
                                ,self.config['classification']['names']
                                ,self.img_pub_flag)

        else:
            print('Error : Please write weights path in yaml file')
            exit()
            
        print('\nLoad weights file : ', weight_path)
        self.bridge = CvBridge()

        rospy.Subscriber(subscribing_image_topic, Image, self.classify)
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
