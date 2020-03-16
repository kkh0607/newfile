#!/usr/bin/env python

import os
import numpy as np
import cv2
import sys
import rospy
import math
import time
from duckietown import DTROS
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage, CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError

class MyNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyNode, self).__init__(node_name=node_name)
        # construct publisher
        self.pub = rospy.Publisher('chatter', String, queue_size=10)
        self.imagesub = rospy.Subscriber("/duckiesam/camera_node/image/compressed", CompressedImage, self.find_marker, buff_size=921600,queue_size=1)
        self.pub_image = rospy.Publisher('~image', Image, queue_size = 1)
        self.bridge = CvBridge()
        self.gotimage = False
        self.imagelast = None
        self.processedImg = None
        self.detected = False


    def find_marker(self, image_msg):
        try:
            self.imagelast = self.bridge.compressed_imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        if self.gotimage == False:
            self.gotimage = True
            
        gray = cv2.cvtColor(self.imagelast, cv2.COLOR_BGR2GRAY)
        
        detection, corners = cv2.findCirclesGrid(gray,(7,3))
        
        self.processedImg = self.imagelast.copy()
        
        if detection:
            cv2.drawChessboardCorners(self.processedImg, (7,3), corners, detection)
            self.detected = True
	else:
	    self.detected = False

    def run(self):
        # publish message every 1 second
        rate = rospy.Rate(1) # 1Hz
        while not rospy.is_shutdown():
            if self.gotimage:
                message = "Hello World! %s" % os.environ['VEHICLE_NAME']
                rospy.loginfo("Publishing message: '%s'" % message)
                self.pub.publish(message)
                if self.detected:
                    img_out = self.bridge.cv2_to_imgmsg(self.processedImg, "bgr8")
                    self.pub_image.publish(img_out)
                    rospy.loginfo("Detected '%s'" % message)
                else:
                    img_out = self.bridge.cv2_to_imgmsg(self.imagelast, "bgr8")
                    self.pub_image.publish(img_out)
                    rospy.loginfo("Not detected '%s'" % message)

            rate.sleep()


if __name__ == '__main__':
    # create the node
    node = MyNode(node_name='my_node')
    # run node
    node.run()
    # keep spinning
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")
    cv2.destroyAllWindows()
    
