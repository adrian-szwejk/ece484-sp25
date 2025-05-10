#!/usr/bin/env python3
import sys
import math
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow:
    def __init__(self):
        # --- hard-coded PID gains ---
        self.kp = 1.0
        self.ki = 0.0
        self.kd = 0.0001

        # --- hard-coded wall-follow params ---
        self.desired_left    = 0.55                  # m
        self.lookahead       = 0.5                   # m
        self.trunc_angle     = math.radians(270)     # Hokuyo FOV in radians
        self.smoothing_size  = 5                     # moving-average half-width

        # speed tiers based on |steering|
        self.error_vels = {
            'low':    4.0,
            'medium': 3.0,
            'high':   2.0,
        }

        # state for PID timing
        self.prev_error = 0.0
        self.integral   = 0.0
        self.prev_time  = rospy.Time.now().to_sec()

        # subscribe to lidar
        self.lidar_sub = rospy.Subscriber(
            '/scan',
            LaserScan,
            self.lidar_callback,
            queue_size=5
        )

        # publish to low-level navigation mux
        self.drive_pub = rospy.Publisher(
            '/vesc/low_level/ackermann_cmd_mux/input/navigation',
            AckermannDriveStamped,
            queue_size=5
        )

    def preprocess_scan(self, scan_msg):
        raw = list(scan_msg.ranges)
        N = len(raw)
        trunc_size = int((self.trunc_angle /
                          (scan_msg.angle_max - scan_msg.angle_min)) * N)
        start = N//2 - trunc_size//2
        end   = N//2 + trunc_size//2
        seg = raw[start:end]

        for i in range(len(seg)):
            if math.isnan(seg[i]):
                seg[i] = 0.0

        s = self.smoothing_size
        smooth = []
        for i in range(s, len(seg)-s):
            window = seg[i-s+1 : i+s]
            smooth.append(sum(window) / float(2*s - 1))
        return smooth

    def get_range_at_filtered(self, filtered, angle, angle_increment):
        corr = angle + (self.trunc_angle/2.0)
        idx = int(math.floor(corr / angle_increment))
        idx = max(0, min(idx, len(filtered)-1))
        return filtered[idx]

    def get_error(self, filtered, angle_increment):
        a, b = 0.5, 1.4
        d_a = self.get_range_at_filtered(filtered, a, angle_increment)
        d_b = self.get_range_at_filtered(filtered, b, angle_increment)
        theta = 0.9

        alpha = math.atan2(d_a*math.cos(theta) - d_b,
                           d_a*math.sin(theta))
        d_t  = d_b * math.cos(alpha)
        d_t1 = d_t + self.lookahead * math.sin(alpha)
        return d_t1 - self.desired_left

    def pid_control(self, error):
        now = rospy.Time.now().to_sec()
        dt  = max(1e-6, now - self.prev_time)

        self.integral  += error
        derivative     = (error - self.prev_error) / dt
        steer          = (self.kp*error +
                          self.kd*derivative +
                          self.ki*self.integral)

        if math.isnan(steer):
            steer = 0.0
        steer = max(-0.4, min(0.4, steer))

        a = abs(steer)
        if a > 0.349:
            speed = self.error_vels['high']
        elif a > 0.174:
            speed = self.error_vels['medium']
        else:
            speed = self.error_vels['low']

        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "laser"
        msg.drive.steering_angle = steer
        msg.drive.speed = speed
        self.drive_pub.publish(msg)

        self.prev_error = error
        self.prev_time  = now

    def lidar_callback(self, data):
        filtered = self.preprocess_scan(data)
        err = self.get_error(filtered, data.angle_increment)
        self.pid_control(err)

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.loginfo("Wall-follow node started:")
    rospy.loginfo(" • /scan → /vesc/low_level/ackermann_cmd_mux/input/navigation")
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
