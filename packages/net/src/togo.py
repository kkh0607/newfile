
import roslib
roslib.load_manifest('net')
import rospy

import tf
import math
from duckietown import DTROS
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage, CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError
from image_geometry import PinholeCameraModel
from duckietown_msgs.msg import Twist2DStamped, Pose2DStamped

class Drive(DTROS):
    def __init__(self, node_name):
    # initialize the DTROS parent class
        super(ToGo, self).__init__(node_name=node_name)
	

    def moving(self):
	sending = tf.TransformBroadcaster()
	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
	    time = rospy.Time.now().to_sec() * math.pi
	    sending.sendTransform((2.0 * math.sin(time), 2.0 * math.cos(t), 0.0), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), "needtogo", "duckieking")
	    rate.sleep()
        

if __name__ == '__main__':
    # create the node
    node = ToGo(node_name='togo')

    node.moving()
    # keep spinning
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")

