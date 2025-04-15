#!/usr/bin/env python3

import rospy
import sys
import select
import termios
import tty
from std_msgs.msg import Float64

keyBindings = {
    'w': (3.0, 0.0),
    'a': (3.0, 0.5),
    'd': (3.0, -0.5),
    's': (-3.0, 0.0),
    'q': (0.0, 0.0)
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('keyboard_teleop_direct', anonymous=True)

    left_rear_pub = rospy.Publisher('/car_1/left_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    right_rear_pub = rospy.Publisher('/car_1/right_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    left_front_pub = rospy.Publisher('/car_1/left_front_wheel_velocity_controller/command', Float64, queue_size=1)
    right_front_pub = rospy.Publisher('/car_1/right_front_wheel_velocity_controller/command', Float64, queue_size=1)

    left_steer_pub = rospy.Publisher('/car_1/left_steering_hinge_position_controller/command', Float64, queue_size=1)
    right_steer_pub = rospy.Publisher('/car_1/right_steering_hinge_position_controller/command', Float64, queue_size=1)

    speed = 0.0
    angle = 0.0

    try:
        print("Control keys: w/a/s/d to drive, q to stop, Ctrl+C to exit")
        while not rospy.is_shutdown():
            key = getKey()
            if key in keyBindings:
                speed, angle = keyBindings[key]

                for pub in [left_rear_pub, right_rear_pub, left_front_pub, right_front_pub]:
                    pub.publish(speed)

                for pub in [left_steer_pub, right_steer_pub]:
                    pub.publish(angle)

            elif key == '\x03':
                break

    finally:
        for pub in [left_rear_pub, right_rear_pub, left_front_pub, right_front_pub]:
            pub.publish(0.0)
        for pub in [left_steer_pub, right_steer_pub]:
            pub.publish(0.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
