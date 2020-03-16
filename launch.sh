#!/bin/bash

set -e

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------
roscore &
sleep 5
rosrun my_package my_node.py
