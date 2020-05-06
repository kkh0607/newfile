#!/bin/bash

set -e

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------
#roslaunch my_package my_draft.launch veh:=duckieking
#rosrun my_package drive_circle.py
echo $ROS_IP
export ROS_MASTER_URI=http://192.168.1.119:11311/
export ROS_IP=192.168.1.155
roslaunch my_package my_draft.launch
