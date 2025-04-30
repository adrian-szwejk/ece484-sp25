#!/usr/bin/env python3
import rospy
import tf
from nav_msgs.msg import Odometry

class VescTfBroadcaster:
    def __init__(self):
        self.br_dynamic = tf.TransformBroadcaster()
        # self.br_static  = tf.StaticTransformBroadcaster()

        # Publish the static base_link → laser transform once
        static_tf = tf.TransformStamped()
        static_tf.header.stamp = rospy.Time.now()
        static_tf.header.frame_id = "base_link"
        static_tf.child_frame_id  = "laser"
        static_tf.transform.translation.x = 0.0
        static_tf.transform.translation.y = 0.0
        static_tf.transform.translation.z = 0.0
        static_tf.transform.rotation.x = 0.0
        static_tf.transform.rotation.y = 0.0
        static_tf.transform.rotation.z = 0.0
        static_tf.transform.rotation.w = 1.0
        # self.br_static.sendTransform(static_tf)

        # Subscribe to /vesc/odom for the dynamic odom→base_link TF
        rospy.Subscriber('/vesc/odom', Odometry, self.handle_odom)

    def handle_odom(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        # Broadcast odom → base_link
        self.br_dynamic.sendTransform(
            (pos.x, pos.y, pos.z),
            (ori.x, ori.y, ori.z, ori.w),
            msg.header.stamp,
            "base_link",  # child_frame
            "odom"        # parent_frame
        )

if __name__ == '__main__':
    rospy.init_node('vesc_tf_broadcaster')
    VescTfBroadcaster()
    rospy.spin()


