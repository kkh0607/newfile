#!/bin/bash

set -e

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------
#roslaunch my_package my_draft.launch veh:=duckieking
#rosrun my_package drive_circle.py
#echo $ROS_IP
#export ROS_MASTER_URI="http://192.168.1.154:11311/"
#export ROS_IP="192.168.1.154"

roslaunch my_package my_draft.launch veh:=duckieking
#roslaunch net net_node.launch
#docker run -it --rm --net host -v /data:/data  /duckietown/dt-ros-commons:daffy-arm32v7 /bin/bash
#docker -H duckieking.local run -it --rm -v /data:/data --net=host duckietown/newfile:v1-arm32v7
