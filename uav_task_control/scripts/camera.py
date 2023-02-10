#!/usr/bin/env python
import rospy
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class RSD435:
    def __init__(self,name):
        self.name = name
        self.topic = '/'+name+'/rsd435/color/image_raw'
        self.colorSub = rospy.Subscriber(self.topic, Image, self._color_cb)
        self.bridge = CvBridge()
        self.check_sensor_ready()
        print("start camera of "+self.topic)

    def _color_cb(self,data):
        try:
            color = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv.imshow(self.name, color)
            cv.waitKey(0)
        except CvBridgeError as e:
            print(e)

    def check_sensor_ready(self):
        rospy.logdebug("waiting for"+self.topic+"to be ready.")
        data = None
        while data is None and not rospy.is_shutdown():
            try:
                data = rospy.wait_for_message(self.topic, Image, timeout=5.0)
                rospy.logdebug("current"+self.topic+"is ready")
            except:
                rospy.logdebug("current"+self.topic+"is not ready, retrying.")
