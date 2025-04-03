#!/bin/bash

# Check if the user has provided a netid as a parameter
if [ -z "$1" ]; then
  echo "Usage: $0 <netid>"
  exit 1
fi

# Set the netid from the parameter
NETID="$1"

# Navigate to the working directory
cd /home/"$NETID"/ece484-sp25 || { echo "Failed to navigate to /home/$NETID/ece484-sp25"; exit 1; }

# Source ROS Noetic setup.bash
source /opt/ros/noetic/setup.bash

# Run catkin_make
catkin_make

# Source the devel/setup.bash
source devel/setup.bash

echo "ROS environment for $NETID is set up successfully!"
