cmake_minimum_required(VERSION 3.0.2)
project(f1tenth_slam)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
    tf2_ros
)

catkin_package(
  CATKIN_DEPENDS rospy tf2_ros
)

# Install config and launch files so that roslaunch / rosparam can find them
install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  src/vesc_odom_tf_broadcaster.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

