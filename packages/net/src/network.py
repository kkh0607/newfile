#!/usr/bin/env python

import roslib
roslib.load_manifest('net')
import rospy

import tf
from duckietown import DTROS
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage, CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError
from image_geometry import PinholeCameraModel
from duckietown_msgs.msg import Twist2DStamped, Pose2DStamped

class Network(DTROS):
    def __init__(self, node_name):
    # initialize the DTROS parent class
        super(Network, self).__init__(node_name=node_name)

	#subscriber to joy_mapper_node/car_cmd
	robot_name = rospy.get_param('~duckie')

	#if king -> wait for sam's detection, initialy pose (0, 0, 0) and (0, 0, 0), (detection, set) bool value should trigger drive_circle.py
	#if sam -> get the most realiable pose for 10 sec, initial (tvecW[2], tvecW[0], angle_l)
	self.sub_pose = rospy.Subscriber('/%s/velocity_to_pose_node/pose' % self.robot_name, Pose2DStamped, self.update_pose, robot_name)

    def update_pose(self, pose_msg, robot_name):
	sending = tf.TransformBroadcaster()
	sending.sendTransform((pose_msg.x, pose_msg.y, 0), tf.transformations.quaternion_from_euler(0, 0, pose_msg.theta), rospy.Time.now(), robot_name, "world")
        

if __name__ == '__main__':
    # create the node
    node = Network(node_name='network')

    # keep spinning
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")

