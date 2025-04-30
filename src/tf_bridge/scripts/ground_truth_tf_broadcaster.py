#!/usr/bin/env python3
import rospy
import tf
from nav_msgs.msg import Odometry

def handle_ground_truth(msg):
    br = tf.TransformBroadcaster()
    pos = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    br.sendTransform(
        (pos.x, pos.y, pos.z),
        (ori.x, ori.y, ori.z, ori.w),
        msg.header.stamp,
        "car_1_base_link",  # child_frame_id
        "odom"              # parent_frame_id
    )

if __name__ == '__main__':
    rospy.init_node('ground_truth_tf_broadcaster')
    rospy.Subscriber('/car_1/ground_truth', Odometry, handle_ground_truth)
    rospy.spin()
