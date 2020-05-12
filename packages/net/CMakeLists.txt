cmake_minimum_required(VERSION 2.8.3)
project(net)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  duckietown_msgs
  rospy
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
