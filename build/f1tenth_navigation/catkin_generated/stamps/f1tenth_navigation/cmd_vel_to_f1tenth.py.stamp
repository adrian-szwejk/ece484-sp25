#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

WHEEL_RADIUS = 0.05

def callback(msg):
    speed = msg.linear.x
    steer = msg.angular.z
    wheel_cmd = speed / WHEEL_RADIUS

    pub_steer_left.publish(Float64(steer))
    pub_steer_right.publish(Float64(steer))

    for wp in wheel_pubs:
        wp.publish(Float64(wheel_cmd))

rospy.init_node("cmd_vel_to_f1tenth")

pub_steer_left = rospy.Publisher("/car_1/left_steering_hinge_position_controller/command", Float64, queue_size=1)
pub_steer_right = rospy.Publisher("/car_1/right_steering_hinge_position_controller/command", Float64, queue_size=1)

wheel_pubs = [
    rospy.Publisher("/car_1/left_front_wheel_velocity_controller/command", Float64, queue_size=1),
    rospy.Publisher("/car_1/right_front_wheel_velocity_controller/command", Float64, queue_size=1),
    rospy.Publisher("/car_1/left_rear_wheel_velocity_controller/command", Float64, queue_size=1),
    rospy.Publisher("/car_1/right_rear_wheel_velocity_controller/command", Float64, queue_size=1),
]

rospy.Subscriber("/cmd_vel", Twist, callback)
rospy.spin()
