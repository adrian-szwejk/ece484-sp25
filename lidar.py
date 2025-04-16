#!usr/bin/env python3

# rostopic list 
# rostopic info /scan
# Slamtoolbox
    # https://github.com/SteveMacenski/slam_toolbox/tree/noetic-devel

# Ros CreatePackage
    # https://wiki.ros.org/ROS/Tutorials/CreatingPackage

# Rosbag 
    # https://wiki.ros.org/rosbag
    #  For record on car to view at home

# Image sensor
    # https://docs.ros.org/en/jade/api/sensor_msgs/html/msg/Image.html

# Laser Scan
    # https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html

# Hector Mapping
    # https://wiki.ros.org/hector_mapping

# Hector Geotiff
    # https://wiki.ros.org/hector_geotiff

# Build map using logged data
    # https://wiki.ros.org/hector_slam/Tutorials/MappingUsingLoggedData

# Hector Slam alternative (gmapping)
    # https://wiki.ros.org/gmapping

# Github
    # https://github.com/tu-darmstadt-ros-pkg/hector_slam
import rospy
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2

def scan_callback(msg):
    min_range = min(msg.ranges)
    max_range = max(msg.ranges)
    rospy.loginfo("Closest obstacle at %f meters", min_range)
    rospy.loginfo("Furthest obstacle at %f meters", max_range)

def img_callback(self, data):
        try:
            # Convert a ROS image message into an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        raw_img = cv_image.copy()
        cv2.imwrite('raw.png', raw_img)
        mask_image, bird_image = self.detection(raw_img)

        if mask_image is not None and bird_image is not None:
            # Convert an OpenCV image into a ROS image message
            out_img_msg = self.bridge.cv2_to_imgmsg(mask_image, 'bgr8')
            out_bird_msg = self.bridge.cv2_to_imgmsg(bird_image, 'bgr8')

            # Publish image message in ROS
            self.pub_image.publish(out_img_msg)
            self.pub_bird.publish(out_bird_msg)

def main():
    rospy.init_node('test')
    # Callback fn to only interrupt car when receives data
    rospy.Subscriber('/scan', LaserScan, scan_callback)
    rospy.Subscriber('D435I/color/image_raw', Image, img_callback, queue_size=1)
    # Inifinite loop = .spin()
    rospy.spin()

if __name__ == '__main__':
    main()