#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

# --- Tunable constants ---
KP = 1.0
KD = 0.001
KI = 0.005     # unused for now
L = 1.0        # look‑ahead distance (m)
MIN_DIST_LEFT = 1.2  # desired clearance from left wall (m)
WHEEL_RADIUS = 0.05   # <— set this to your wheel radius in meters

# --- State ---
prev_error = 0.0
integral = 0.0
last_time = None

# Publishers (to be initialized in main)
left_steering_pub = None
right_steering_pub = None
wheel_velocity_pubs = []

def scan_callback(scan: LaserScan):
    global prev_error, integral, last_time

    if rospy.is_shutdown():
        return

    if left_steering_pub is None or right_steering_pub is None or not wheel_velocity_pubs:
        rospy.logwarn("Publishers not ready or shutdown in progress.")
        return

    # compute indices for 45° and 90°
    ang_a = math.radians(45.0)
    ang_b = math.radians(90.0)
    idx_a = int((ang_a - scan.angle_min) / scan.angle_increment)
    idx_b = int((ang_b - scan.angle_min) / scan.angle_increment)

    # read distances, cap infinities
    dist_a = scan.ranges[idx_a]
    dist_b = scan.ranges[idx_b]
    if math.isinf(dist_a) or math.isnan(dist_a): dist_a = scan.range_max
    if math.isinf(dist_b) or math.isnan(dist_b): dist_b = scan.range_max

    # wall‑angle α and projected distance dt1
    alpha = math.atan2(
        dist_a * math.cos(ang_b - ang_a) - dist_b,
        dist_a * math.sin(ang_b - ang_a)
    )
    dt = dist_b * math.cos(alpha)
    dt1 = dt + L * math.sin(alpha)

    # PID error & timing
    error = MIN_DIST_LEFT - dt1
    now = rospy.Time.now().to_sec()
    dt_seconds = (now - last_time) if last_time else 0.0
    integral += error * dt_seconds

    # PID steering command
    d_error = (error - prev_error) / dt_seconds if dt_seconds > 0 else 0.0
    steer = -(KP * error + KD * d_error)  # + KI * integral if you enable I-term

    # speed schedule
    abs_steer = abs(steer)
    if abs_steer > math.radians(20):
        speed = 0.3
    elif abs_steer > math.radians(10):
        speed = 0.5
    else:
        speed = 0.5

    # --- Publish to low-level controllers ---
    # 1) Steering hinges take angle commands directly
    left_steering_pub.publish(Float64(steer))
    right_steering_pub.publish(Float64(steer))

    # 2) Wheel velocity controllers expect angular velocity (rad/s)
    wheel_cmd = speed / WHEEL_RADIUS
    for wp in wheel_velocity_pubs:
        wp.publish(Float64(wheel_cmd))

    # update state
    prev_error = error
    last_time = now

def shutdown():
    rospy.loginfo("Stopping car...")
    if left_steering_pub and right_steering_pub:
        left_steering_pub.publish(Float64(0.0))
        right_steering_pub.publish(Float64(0.0))
    if wheel_velocity_pubs:
        for wp in wheel_velocity_pubs:
            wp.publish(Float64(0.0))

def main():
    global left_steering_pub, right_steering_pub, wheel_velocity_pubs, last_time

    rospy.init_node('wall_follow_lowlevel_py')
    rospy.loginfo("Starting low‑level wall_follow node")

    # Steering hinge publishers
    left_steering_pub = rospy.Publisher(
        '/car_1/left_steering_hinge_position_controller/command',
        Float64, queue_size=1)
    right_steering_pub = rospy.Publisher(
        '/car_1/right_steering_hinge_position_controller/command',
        Float64, queue_size=1)

    # Wheel velocity publishers
    wheel_velocity_pubs = [
        rospy.Publisher('/car_1/left_rear_wheel_velocity_controller/command', Float64, queue_size=1),
        rospy.Publisher('/car_1/right_rear_wheel_velocity_controller/command', Float64, queue_size=1),
        rospy.Publisher('/car_1/left_front_wheel_velocity_controller/command', Float64, queue_size=1),
        rospy.Publisher('/car_1/right_front_wheel_velocity_controller/command', Float64, queue_size=1),
    ]

    # initialize timing
    last_time = rospy.Time.now().to_sec()

    # LaserScan subscriber
    rospy.Subscriber('/car_1/scan', LaserScan, scan_callback, queue_size=1)

    rospy.on_shutdown(shutdown)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
