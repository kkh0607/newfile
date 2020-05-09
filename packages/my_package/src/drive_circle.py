#!/usr/bin/env python

import os
import numpy as np
import cv2
import sys
import rospy
import math
import time
from duckietown import DTROS
from duckietown_msgs.msg import Twist2DStamped, BoolStamped

class DriveCircle(DTROS):
    def __init__(self, node_name):
    # initialize the DTROS parent class
        super(DriveCircle, self).__init__(node_name=node_name)
        
        self.startingtime = rospy.Time.now()
        self.defaultvelocity = 0.20
        self.defaultomega = -0.35
        
        
        self.rate = rospy.Rate(30)
        
        #publisher to joy_mapper_node/car_cmd
        self.pub_move = rospy.Publisher('~car_cmd', Twist2DStamped, queue_size = 1)
        #init, start
        car_control_msg = Twist2DStamped()
        car_control_msg.v = 0
        car_control_msg.omega = 0
        self.pub_move.publish(car_control_msg)
        rospy.sleep(1)
        
	#subscriber
        
        
        rospy.on_shutdown(self.my_shutdown)
        

    def drive_line(self):
        car_control_msg = Twist2DStamped()
        car_control_msg.header.stamp = rospy.Time.now()
        car_control_msg.v = self.defaultvelocity
        car_control_msg.omega = 0.0
        self.pub_move.publish(car_control_msg)
        
     
    def turn_right(self):
        car_control_msg = Twist2DStamped()
	car_control_msg.header.stamp = rospy.Time.now()
        car_control_msg.v = 0.0
        car_control_msg.omega = -0.5
        self.pub_move.publish(car_control_msg)
        
    def drive_rectangle(self):
        for i in range(0,4):
            self.drive_line()
	    rospy.sleep(5)
            self.turn_right()
	    rospy.sleep(3)
        self.stop()
    
    def drive_circle(self):
        car_control_msg = Twist2DStamped()
        car_control_msg.v = self.defaultvelocity
        car_control_msg.omega = self.defaultomega
        self.pub_move.publish(car_control_msg)
	#self.rate.sleep()
        
    def stop(self):
        car_control_msg = Twist2DStamped()
        car_control_msg.v = 0
        car_control_msg.omega = 0
        self.pub_move.publish(car_control_msg)
        rospy.sleep(5)

    def drive(self):
        while not rospy.is_shutdown():
            #self.drive_rectangle()
	    #self.stop()
            self.drive_circle()
	self.rate.sleep()
               
    def my_shutdown(self):
	self.stop()
	rospy.sleep(1)
	print("shutting down")
        
        
if __name__ == '__main__':
    # create the node
    node = DriveCircle(node_name='drive_circle')
    # run node
    node.drive()
    # keep spinning
    #try:
    rospy.spin()
    #except KeyboardInterrupt:
        #print("shutting down")

    #not needed to destoroy windows, no windows
    #cv2.destroyAllWindows()
