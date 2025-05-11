#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

# --- Tunable constants ---
KP = 1.0
KD = 0.001
KI = 0.005     # unused for now
L = 1.0        # look-ahead distance (m)
MIN_DIST_LEFT = 0.5  # desired clearance from left wall (m)
WHEEL_RADIUS = 0.05   # set this to your wheel radius in meters

# --- State ---
prev_error = 0.0
integral = 0.0
last_time = None

# Publisher (to be initialized in main)
control_pub = None

def scan_callback(scan: LaserScan):
    global prev_error, integral, last_time, control_pub

    if rospy.is_shutdown():
        return

    if control_pub is None:
        rospy.logwarn("Publisher not ready yet.")
        return

    # compute indices for 45° and 90°
    ang_a = math.radians(45.0)
    ang_b = math.radians(90.0)
    idx_a = int((ang_a - scan.angle_min) / scan.angle_increment)
    idx_b = int((ang_b - scan.angle_min) / scan.angle_increment)

    # read distances, cap infinities
    dist_a = scan.ranges[idx_a]
    dist_b = scan.ranges[idx_b]
    if math.isinf(dist_a) or math.isnan(dist_a):
        dist_a = scan.range_max
    if math.isinf(dist_b) or math.isnan(dist_b):
        dist_b = scan.range_max

    # Debug print raw distances
    rospy.loginfo(f"Raw distances - a: {dist_a:.2f}m, b: {dist_b:.2f}m")

    # wall-angle α and projected distance dt1
    alpha = math.atan2(
        dist_a * math.cos(ang_b - ang_a) - dist_b,
        dist_a * math.sin(ang_b - ang_a)
    )
    dt = dist_b * math.cos(alpha)
    dt1 = dt + L * math.sin(alpha)

    # PID error & timing
    error = MIN_DIST_LEFT - dt1
    now = rospy.Time.now().to_sec()
    dt_sec = (now - last_time) if last_time else 0.0
    integral += error * dt_sec
    d_error = (error - prev_error) / dt_sec if dt_sec > 0 else 0.0

    # compute steering angle
    steer = -(KP * error + KD * d_error)  # add + KI*integral if you enable I-term

    # speed schedule based on steering magnitude
    abs_st = abs(steer)
    if abs_st > math.radians(20):
        speed = 0.75  # m/s
    else:
        speed = 1.5  # m/s

    # Create and publish Ackermann message
    drive_msg = AckermannDriveStamped()
    drive_msg.header.stamp = rospy.Time.now()
    drive_msg.drive.steering_angle = steer
    drive_msg.drive.speed = speed

    # Debug print before publishing
    rospy.loginfo(f"Publishing - Speed: {speed:.2f}m/s, Steering: {math.degrees(steer):.1f}°")
    control_pub.publish(drive_msg)

    # update state
    prev_error = error
    last_time = now

def shutdown():
    rospy.loginfo("Stopping car...")
    if control_pub:
        stop_msg = AckermannDriveStamped()
        stop_msg.header.stamp = rospy.Time.now()
        stop_msg.drive.speed = 0.0
        stop_msg.drive.steering_angle = 0.0
        control_pub.publish(stop_msg)

def main():
    global control_pub, last_time

    rospy.init_node('wall_follow_lowlevel_car')
    rospy.loginfo("Starting modified wall_follow node with AckermannDriveStamped")

    # Single publisher for both steering and speed
    control_pub = rospy.Publisher(
        '/vesc/low_level/ackermann_cmd_mux/input/navigation',
        AckermannDriveStamped,
        queue_size=1
    )

    # Wait for publisher to register
    rospy.sleep(0.5)

    # initialize timing
    last_time = rospy.Time.now().to_sec()

    # LaserScan subscriber
    rospy.Subscriber('/scan', LaserScan, scan_callback, queue_size=1)

    rospy.on_shutdown(shutdown)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
