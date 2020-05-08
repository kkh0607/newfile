#!/usr/bin/env python

import os
import numpy as np
import cv2
import sys
import rospy
import math
import time
import rosbag
from duckietown import DTROS
from std_msgs.msg import String, Int32
from sensor_msgs.msg import CompressedImage, CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError
from image_geometry import PinholeCameraModel
from duckietown_msgs.msg import Twist2DStamped, BoolStamped

class LeaderDrive(DTROS):
    def __init__(self, node_name):
    # initialize the DTROS parent class
        super(LeaderDrive, self).__init__(node_name=node_name)
        self.count = 0
        self.max = 100
        self.on = False
        self.pub_move = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size = 1)
        self.sub_request = rospy.Subscriber("~request", Int32, self.bagplay, queue_size=1)
        self.starting_time = rospy.Time.now()
        self.pub_on = rospy.Publisher("~on", BoolStamped, queue_size = 1)
        
    #def move(self):
        #publish movement
        #cmd = Twist2DStamped()
        #cmd.header.stamp = rospy.Time.now()
        
    
    def start(self):
        self.sub_move = rospy.Subscriber("/duckiesam/joy_mapper_node/car_cmd", Twist2DStamped, self.bagout queue_size = 1)
    
    def bagout(self):
        #publish bag
        bag = rosbag.Bag('bag'+self.count + '.bag','w')
        
    def bagplay(self):
        

if __name__ == '__main__':
    # create the node
    node = LeaderDrive(node_name='leader_drive')

    # keep spinning
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")

    cv2.destroyAllWindows()
