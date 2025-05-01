#!/bin/bash

source /opt/ros/noetic/setup.bash

catkin_make

source devel/setup.bash

echo "ROS environment set up successfully!"