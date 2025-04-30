cmake_minimum_required(VERSION 3.0.2)
project(f1tenth_navigation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  move_base
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/cmd_vel_to_f1tenth.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
