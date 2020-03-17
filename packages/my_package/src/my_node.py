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
from image_geometry import PinholeCameraModel


class MyNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyNode, self).__init__(node_name=node_name)
        # construct publisher
        self.pub = rospy.Publisher('chatter', String, queue_size=10)
        self.imagesub = rospy.Subscriber("/duckiesam/camera_node/image/compressed", CompressedImage, self.find_marker, buff_size=921600,queue_size=1)
        self.pub_image = rospy.Publisher('~image', Image, queue_size = 1)
        self.infosub = rospy.Subscriber("/duckiesam/camera_node/camera_info", CameraInfo, self.get_camera_info, queue_size=1)
        self.camerainfo = PinholeCameraModel()
        self.bridge = CvBridge()
        self.gotimage = False
        self.imagelast = None
        self.processedImg = None
        self.detected = False
        self.solP = False
        self.rotationvector = None
        self.translationvector = None
	self.axis = np.float32([[0.0125,0,0], [0,0.0125,0], [0,0,-0.0375]]).reshape(-1,3)

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
            
        gray = cv2.cvtColor(self.imagelast, cv2.COLOR_BGR2GRAY)
        
        detection, corners = cv2.findCirclesGrid(gray,(7,3))
        
        self.processedImg = self.imagelast.copy()
        
        if detection:
            cv2.drawChessboardCorners(self.processedImg, (7,3), corners, detection)
            self.detected = True
            
            twoone = []
            for i in range(0, 21):
                point = [corners[i][0][0], corners[i][0][1]]
                twoone.append(point)
            twoone = np.array(twoone)
            
            self.originalmatrix()
            self.gradient(twoone)
	    self.detected = self.solP
            
        else:
            self.detected = False
            
    #step 2 : makes matrix for 3d original shape
    def originalmatrix(self):
    #coners and points
        self.originalmtx = np.zeros([21, 3])
        for i in range(0, 7):
            for j in range(0, 3):
                self.originalmtx[i + j * 7, 0] = 0.0125 * i - 0.0125 * 3
                self.originalmtx[i + j * 7, 1] = 0.0125 * j - 0.0125

    
    #step 3 : use intrinsic matrix and solvePnP, return rvec and tvec, print axis
    def gradient(self, imgpts):
    #using solvePnP to find rotation vector and translation vector and also find 3D point to the image plane
        self.solP, self.rotationvector, self.translationvector = cv2.solvePnP(self.originalmtx, imgpts, self.camerainfo.intrinsicMatrix(), self.camerainfo.distortionCoeffs())
	if self.solP:
	    pointsin3D, jacoB = cv2.projectPoints(self.originalmtx, self.rotationvector, self.translationvector, self.camerainfo.intrinsicMatrix(), self.camerainfo.distortionCoeffs())
	    pointaxis, _ = cv2.projectPoints(self.axis, self.rotationvector, self.translationvector, self.camerainfo.intrinsicMatrix(), self.camerainfo.distortionCoeffs())
	    self.processedImg = cv2.line(self.processedImg, tuple(imgpts[10].ravel()), tuple(pointaxis[0].ravel()), (255, 0, 0), 2)
	    self.processedImg = cv2.line(self.processedImg, tuple(imgpts[10].ravel()), tuple(pointaxis[1].ravel()), (0, 255, 0), 2)
	    self.processedImg = cv2.line(self.processedImg, tuple(imgpts[10].ravel()), tuple(pointaxis[2].ravel()), (0, 0, 255), 3)

        
    def run(self):
        # publish message every 1 second
        rate = rospy.Rate(1) # 1Hz
        while not rospy.is_shutdown():
            if self.gotimage:
                message = "%s" % os.environ['VEHICLE_NAME']
                if self.detected:
                    img_out = self.bridge.cv2_to_imgmsg(self.processedImg, "bgr8")
                    self.pub_image.publish(img_out)
                    rospy.loginfo("Detected '%s'" % message)
                    self.pub.publish(message)
                else:
                    img_out = self.bridge.cv2_to_imgmsg(self.imagelast, "bgr8")
                    self.pub_image.publish(img_out)
                    rospy.loginfo("Not detected '%s'" % message)
                    self.pub.publish(message)
            
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
    
