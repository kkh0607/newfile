#!/usr/bin/env python

import roslib
roslib.load_manifest('net')
import rospy
import math
import tf
import os
import numpy as np
import cv2
import sys
import time
from duckietown import DTROS
from std_msgs.msg import String, Int32
from sensor_msgs.msg import CompressedImage, CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError
from image_geometry import PinholeCameraModel
from duckietown_msgs.msg import Twist2DStamped, BoolStamped, VehiclePose, Pose2DStamped

class FollowerDrive(DTROS):
    def __init__(self, node_name):
    # initialize the DTROS parent class
        super(FollowerDrive, self).__init__(node_name=node_name)
        
	self.listener = tf.TransformListener()
	#self.sub_follower_pose = rospy.Subscriber('', VehiclePose)
        self.sub_image = rospy.Subscriber("/duckiesam/camera_node/image/compressed", CompressedImage, self.find_marker, buff_size=921600,queue_size=1)
        self.pub_image = rospy.Publisher('/duckiesam/modified_image', Image, queue_size = 1)
        self.sub_info = rospy.Subscriber("/duckiesam/camera_node/camera_info", CameraInfo, self.get_camera_info, queue_size=1)
        self.pub_move = rospy.Publisher("/duckiesam/joy_mapper_node/car_cmd", Twist2DStamped, queue_size = 1)
	self.pub_pose = rospy.Publisher("/duckiesam/velocity_to_pose_node/pose", Pose2DStamped, queue_size=1)
	self.pub_detection = rospy.Publisher("detect", BoolStamped, queue_size=1)

	#values for detecting marker
        self.starting = 0
        self.ending = 0
        self.camerainfo = PinholeCameraModel()
        self.bridge = CvBridge()
        self.gotimage = False
        self.imagelast = None
        self.processedImg = None
        self.detected = False
	self.number = 0

	#values for calculating pose of robot
	self.originalmatrix()
        self.solP = False
        #self.rotationvector = None
        #self.translationvector = None
        self.axis = np.float32([[0.0125,0,0], [0,0.0125,0], [0,0,-0.0375]]).reshape(-1,3)
        #self.distance = None
        #self.angle_f = None
        #self.angle_l = None
	self.count = 0
	self.add_distance = 0.0
	self.add_x = 0.0
	self.add_y = 0.0
	self.add_angle_l = 0.0
	self.needed = True
	#values for driving the robot
        self.initialvalues()

	rospy.on_shutdown(self.my_shutdown)
        
    def initialvalues(self):
        
        self.maxdistance = 0.25
        self.speedN = 0
        self.e_vB = 0
        self.rotationN = 0
        self.mindistance = 0.2
        self.d_e = 0 #distance error
        self.d_e_1 = 0
        self.d_e_2 = 0
        self.y2 = 0
        self.controltime = rospy.Time.now()
        self.Kp = 5
        self.Ki = 0
        self.Kd = 0
        self.I = 0
        self.r1 = 3
        self.r2 = 0
        self.r3 = 0
        
        
    #get camera info for pinhole camera model
    def get_camera_info(self, camera_msg):
        self.camerainfo.fromCameraInfo(camera_msg)

    #step 1 : find the back circle grids using cv2.findCirclesGrid
    ##### set (x,y) for points and flag for detection
    def find_marker(self, image_msg):
        try:
            self.imagelast = self.bridge.compressed_imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        if self.gotimage == False:
            self.gotimage = True
         
        #time checking
        self.starting = rospy.Time.now()
        #from_last_image = (self.starting - self.ending).to_sec()
        
        gray = cv2.cvtColor(self.imagelast, cv2.COLOR_BGR2GRAY)
        
        detection, corners = cv2.findCirclesGrid(gray,(7,3))
        
        processedImg = self.imagelast.copy()
	
	if detection:
	    cv2.drawChessboardCorners(processedImg, (7,3), corners, detection)
            self.detected = True
            
            twoone = []
            for i in range(0, 21):
                point = [corners[i][0][0], corners[i][0][1]]
                twoone.append(point)
            twoone = np.array(twoone)
            
            rotationvector, translationvector, processedImg = self.gradient(twoone,processedImg)
            self.detected = self.solP
            img_out = self.bridge.cv2_to_imgmsg(processedImg, "bgr8")
            self.pub_image.publish(img_out)
            distance, x, y, angle_l = self.find_distance(translationvector, rotationvector)
            
            self.ending = rospy.Time.now()
        else:
            self.detected = False
            img_out = self.bridge.cv2_to_imgmsg(self.imagelast, "bgr8")
            self.pub_image.publish(img_out)
            self.ending = rospy.Time.now()
            #cmd.v = 0
            #cmd.omega = 0
            #self.pub_move.publish(cmd)
	    

    #step 2 : makes matrix for 3d original shape
    def originalmatrix(self):
    #coners and points
        self.originalmtx = np.zeros([21, 3])
        for i in range(0, 7):
            for j in range(0, 3):
                self.originalmtx[i + j * 7, 0] = 0.0125 * i - 0.0125 * 3
                self.originalmtx[i + j * 7, 1] = 0.0125 * j - 0.0125

    
    #step 3 : use intrinsic matrix and solvePnP, return rvec and tvec, print axis
    def gradient(self, imgpts, processedImg):
    #using solvePnP to find rotation vector and translation vector and also find 3D point to the image plane
        self.solP, rotationvector, translationvector = cv2.solvePnP(self.originalmtx, imgpts, self.camerainfo.intrinsicMatrix(), self.camerainfo.distortionCoeffs())
        if self.solP:
            pointsin3D, jacoB = cv2.projectPoints(self.originalmtx, rotationvector, translationvector, self.camerainfo.intrinsicMatrix(), self.camerainfo.distortionCoeffs())
            pointaxis, _ = cv2.projectPoints(self.axis, rotationvector, translationvector, self.camerainfo.intrinsicMatrix(), self.camerainfo.distortionCoeffs())
            processedImg = cv2.line(processedImg, tuple(imgpts[10].ravel()), tuple(pointaxis[0].ravel()), (255, 0, 0), 2)
            processedImg = cv2.line(processedImg, tuple(imgpts[10].ravel()), tuple(pointaxis[1].ravel()), (0, 255, 0), 2)
            processedImg = cv2.line(processedImg, tuple(imgpts[10].ravel()), tuple(pointaxis[2].ravel()), (0, 0, 255), 3)
	    
	return rotationvector, translationvector, processedImg

    #step 4 : find distance between robot and following robot print out distance and time
    def find_distance(self, translationvector, rotationvector):
    #use tvec to calculate distance
        tvx = translationvector[0]
        tvy = translationvector[1]
        tvz = translationvector[2]
        
        distance = math.sqrt(tvx*tvx + tvz*tvz)
        angle_f = np.arctan2(tvx[0],tvz[0])

        R, _ = cv2.Rodrigues(rotationvector)
        R_inverse = np.transpose(R)
        angle_l = np.arctan2(-R_inverse[2,0], math.sqrt(R_inverse[2,1]**2 + R_inverse[2,2]**2))
        
        T = np.array([-np.sin(angle_l), np.cos(angle_l)])

	#tvecW is position of camera(follower vehicle) in world frame
        tvecW = -np.dot(R_inverse, translationvector)

	#desire point [0.20, 0] x,y, now tvz = x tvx = y
        x = tvecW[2][0]
        y = tvecW[0][0]
        
	#y2 = tvecW[0][0] - 0.05*np.sin(angle_l)
        
        textdistance = "Distance = %s, Angle of Follower = %s, Angle of Leader = %s" % (distance, angle_f, angle_l)
        rospy.loginfo("%s" % textdistance)
	
	if self.count < 10 and self.needed:
	    self.add_distance = self.add_distance + distance
	    self.add_x = self.add_x + x
	    self.add_y = self.add_y + y
	    self.add_angle_l = self.add_angle_l
	    self.count += 1
        
	return distance, x, y, angle_l
    
    def initial_pose(self):
	distance = self.add_distance/self.count
	x = self.add_x / self.count
	y = self.add_y / self.count
	angle_l = self.add_angle_l / self.count
	self.count = 0
	pose_init_msg = Pose2DStamped()
	pose_init_msg.x = x
	pose_init_msg.y = y
	pose_init_msg.theta = angle_l
	self.pub_pose.publish(pose_init_msg)
	self.needed = False
	ready = BoolStamped()
	ready.header.stamp = rospy.Time.now()
	ready.data = True
	self.pub_detection.publish(ready)


    def move(self):
        #from the bag move drive
	rate = rospy.Rate(10.0)
        self.listener.waitForTransform("/duckiesam", "/needtogo", rospy.Time(), rospy.Duration(4.0))
	while not rospy.is_shutdown():
	    try:
		now = rospy.Time.now()
		self.listener.waitForTransform("/duckiesam", "/needtogo", now, rospy.Duration(4.0))
		(trans, rot) = self.listener.lookupTransform('/duckiesam', '/needtogo', rospy.Time(0))
	    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		continue

	    angular = 4* math.atan2(trans[1], trans[0])
	    linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
	    cmd = Twist2DStamped()
	    self.number += 1
	    cmd.header.seq = self.number
	    cmd.v = linear
	    cmd.omega = angular
            self.pub_move.publish(cmd)

            rate.sleep()

if __name__ == '__main__':
    # create the node
    node = FollowerDrive(node_name='follower_drive')
    
    if node.count == 10 and node.needed:
	node.initial_pose()

    node.move()
    # keep spinning
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")

    #cv2.destroyAllWindows()
