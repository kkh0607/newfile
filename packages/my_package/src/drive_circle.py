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
        
        self.time = rospy.Time.now()
        self.defaultvelocity = 0.20
        self.defaultomega = -0.40
        self.number = 1
        self.mode = "not set"
        self.rate = rospy.Rate(5)
	self.twistangle = self.defaultomega
	self.twistcount = 0      

        #publisher to joy_mapper_node/car_cmd
        self.pub_move = rospy.Publisher('~car_cmd', Twist2DStamped, queue_size = 1)
        #self.on = False
        
        #subscriber
	#self.sub_follower = rospy.Subscriber('detect', BoolStamped, self.drive, queue_size = 1)

	#init, start
        car_control_msg = Twist2DStamped()
	car_control_msg.header.seq = self.number
        car_control_msg.v = 0
        car_control_msg.omega = 0
        self.pub_move.publish(car_control_msg)
        rospy.sleep(1)
        
        rospy.on_shutdown(self.my_shutdown)
        

    def drive_line(self):
	self.mode = "line"
        car_control_msg = Twist2DStamped()
        self.time = rospy.Time.now()
	car_control_msg.header.stamp = self.time
        car_control_msg.v = self.defaultvelocity
        car_control_msg.omega = -0.4
	if self.number >= 60 and self.number <= 67:
	    car_control_msg.omega = -0.60
	    car_control_msg.v = 0.0
	self.number += 1
	car_control_msg.header.seq = self.number
        self.pub_move.publish(car_control_msg)
	textdistance = "Velocity = %s, Angle = %s" % (car_control_msg.v, car_control_msg.omega)
        rospy.loginfo("%s" % textdistance)
	textdistance = "Num = %s, time = %s, Mode = %s" % (self.number, self.time, self.mode)
        rospy.loginfo("%s" % textdistance)
	#rospy.sleep(0.1)
        
     
    def turn_right(self):
	self.mode = "right"
        car_control_msg = Twist2DStamped()
        self.time = rospy.Time.now()
	car_control_msg.header.stamp = self.time
        car_control_msg.v = 0.0
        car_control_msg.omega = -0.35
	self.number += 1
	car_control_msg.header.seq = self.number
        self.pub_move.publish(car_control_msg)
	textdistance = "Velocity = %s, Angle = %s" % (car_control_msg.v, car_control_msg.omega)
        rospy.loginfo("%s" % textdistance)
	textdistance = "Num = %s, time = %s, Mode = %s" % (self.number, self.time, self.mode)
        rospy.loginfo("%s" % textdistance)
	#rospy.sleep(0.1)
	
    def turn_left(self):
	self.mode = "left"
        car_control_msg = Twist2DStamped()
        self.time = rospy.Time.now()
	car_control_msg.header.stamp = self.time
        car_control_msg.v = 0.0
        car_control_msg.omega = 0.393
	self.number += 1
	car_control_msg.header.seq = self.number
        self.pub_move.publish(car_control_msg)
	textdistance = "Velocity = %s, Angle = %s" % (car_control_msg.v, car_control_msg.omega)
        rospy.loginfo("%s" % textdistance)
	textdistance = "Num = %s, time = %s, Mode = %s" % (self.number, self.time, self.mode)
        rospy.loginfo("%s" % textdistance)
	#self.rate.sleep()
        
    def drive_rectangle(self):
        for i in range(0,70):
            self.drive_line()
	    self.rate.sleep()
	for i in range(0,14):
            self.turn_right()
	    self.rate.sleep()
        #self.stop()
    
    def drive_circle(self):
	self.mode = "circle"
        car_control_msg = Twist2DStamped()
        car_control_msg.v = self.defaultvelocity
        car_control_msg.omega = self.defaultomega - 0.03
	self.number += 1
	self.time = rospy.Time.now()
	car_control_msg.header.stamp = self.time
	car_control_msg.header.seq = self.number
        self.pub_move.publish(car_control_msg)
	textdistance = "Velocity = %s, Angle = %s" % (car_control_msg.v, car_control_msg.omega)
        rospy.loginfo("%s" % textdistance)
	self.rate.sleep()
        
    def drive_twist(self):
	self.mode = "twist"
	car_control_msg = Twist2DStamped()
        self.time = rospy.Time.now()
	car_control_msg.header.stamp = self.time
        car_control_msg.v = self.defaultvelocity
	car_control_msg.omega = self.twistangle
        self.omega()
	self.number += 1
	self.twistcount += 1
	car_control_msg.header.seq = self.number
        self.pub_move.publish(car_control_msg)
	textdistance = "Velocity = %s, Angle = %s" % (car_control_msg.v, car_control_msg.omega)
        rospy.loginfo("%s" % textdistance)
	self.rate.sleep()
	
    def omega(self):
	if self.twistcount == 15:
	    self.twistangle = -self.twistangle
	    self.twistcount = 0
        
    def stop(self):
	self.mode = "stop"
        car_control_msg = Twist2DStamped()
        car_control_msg.v = 0
        car_control_msg.omega = 0
	self.time = rospy.Time.now()
	car_control_msg.header.stamp = self.time
	self.number += 1
	car_control_msg.header.seq = self.number
        self.pub_move.publish(car_control_msg)
        rospy.sleep(5)

    def drive(self):
        while not rospy.is_shutdown():
            #comment out other mode
	    #if self.on:
	    #self.drive_rectangle()
            	#self.stop()
            	#self.drive_circle()
	    	#self.drive_twist()
	    self.drive_line()
	    textdistance = "Num = %s, time = %s, Mode = %s" % (self.number, self.time, self.mode)
            rospy.loginfo("%s" % textdistance)
            rospy.Rate(10).sleep()
    
    #def ready(self, msg):
	#self.on = msg.data

           
    def my_shutdown(self):
        self.stop()
	rospy.sleep(0.5)
        print("shutting down")
	rospy.signal_shutdown("exit")
	
        
if __name__ == '__main__':
    # create the node
    node = DriveCircle(node_name='drive_circle')
    # run node
    node.drive()
    # keep spinning
    try:
	rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")

    #not needed to destoroy windows, no windows
    #cv2.destroyAllWindows()
