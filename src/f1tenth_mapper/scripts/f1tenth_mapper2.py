#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
import tf
from message_filters import Subscriber, ApproximateTimeSynchronizer

INTENSITY_THRESHOLD = 5.0

class F1TENTHMapper:
    def __init__(self):
        rospy.init_node('f1tenth_mapper')

        # --- map parameters ---
        self.resolution = 0.05     # m/cell
        self.size       = 600      # cells → 30×30 m
        self.half_size  = (self.size * self.resolution) / 2.0
        self.map        = np.full((self.size, self.size), -1, dtype=np.int8)

        # origin of world coords
        self.start_x = None
        self.start_y = None

        # latest odom pose
        self.current_x   = None
        self.current_y   = None
        self.current_yaw = None

        # trajectory history (corrected poses)
        self.trajectory = []

        # --- scan-matcher search window ---
        self.dxs     = np.linspace(-0.05, +0.05, 5)
        self.dys     = np.linspace(-0.05, +0.05, 5)
        self.dthetas = np.linspace(-0.03, +0.03, 7)

        # --- time-synced subscribers ---
        odom_sub = Subscriber('/vesc/odom',   Odometry)
        scan_sub = Subscriber('/scan',        LaserScan)
        ats      = ApproximateTimeSynchronizer(
                       [scan_sub, odom_sub],
                       queue_size=10,
                       slop=0.1
                   )
        ats.registerCallback(self.synced_callback)

        # --- map publisher & timer ---
        self.map_pub = rospy.Publisher('/f1tenth_map', OccupancyGrid, queue_size=1)
        rospy.Timer(rospy.Duration(1.0), self.publish_map)

        rospy.loginfo('F1TENTH Mapper started — publishing /f1tenth_map at 1 Hz')
        rospy.spin()


    def odom_callback(self, msg):
        # extract pose straight from /vesc/odom
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        q  = msg.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w]
        )

        # on first message, set map origin reference
        if self.start_x is None:
            self.start_x = px
            self.start_y = py

        self.current_x   = px
        self.current_y   = py
        self.current_yaw = yaw


    def synced_callback(self, scan: LaserScan, odom: Odometry):
        # update our stored odom pose
        self.odom_callback(odom)
        if self.current_x is None:
            return  # still waiting for first odom

        # --- build local scan points ---
        ranges = np.array(scan.ranges)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        valid  = np.logical_and(np.isfinite(ranges), ranges < scan.range_max)
        if len(scan.intensities) == len(ranges):
            valid = np.logical_and(valid,
                       np.array(scan.intensities) > INTENSITY_THRESHOLD)
        angles = angles[valid]
        ranges = ranges[valid]
        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)
        local_pts = np.vstack((xs, ys))    # shape (2, N)

        tx  = self.current_x
        ty  = self.current_y
        yaw = self.current_yaw

        # --- brute-force correlation matcher ---
        best_score  = -1
        best_dx     = 0.0
        best_dy     = 0.0
        best_dtheta = 0.0

        for dtheta in self.dthetas:
            c_ = np.cos(yaw + dtheta)
            s_ = np.sin(yaw + dtheta)
            R  = np.array([[ c_, -s_],
                           [ s_,  c_]])
            world_pts = R.dot(local_pts)  # still relative to (0,0)

            for dx in self.dxs:
                for dy in self.dys:
                    # shift into world coords
                    wp = world_pts + np.array([[tx + dx],
                                               [ty + dy]])
                    xi = ((wp[0] - self.start_x + self.half_size)
                          / self.resolution).astype(int)
                    yi = ((wp[1] - self.start_y + self.half_size)
                          / self.resolution).astype(int)

                    mask = (xi >= 0) & (xi < self.size) \
                         & (yi >= 0) & (yi < self.size)
                    xi2, yi2 = xi[mask], yi[mask]
                    score = np.sum(self.map[yi2, xi2] == 100)
                    if score > best_score:
                        best_score  = score
                        best_dx     = dx
                        best_dy     = dy
                        best_dtheta = dtheta

        # corrected pose = odom + best offset
        tx_corr  = tx + best_dx
        ty_corr  = ty + best_dy
        yaw_corr = yaw + best_dtheta

        # store corrected trajectory
        self.trajectory.append((tx_corr, ty_corr))

        # --- ray-trace each beam into the grid ---
        c2 = np.cos(yaw_corr)
        s2 = np.sin(yaw_corr)
        R2 = np.array([[ c2, -s2],
                       [ s2,  c2]])
        pts_world = R2.dot(local_pts) \
                    + np.array([[tx_corr],[ty_corr]])

        x0 = int((tx_corr - self.start_x + self.half_size) / self.resolution)
        y0 = int((ty_corr - self.start_y + self.half_size) / self.resolution)

        for x_w, y_w in zip(pts_world[0], pts_world[1]):
            xi = int((x_w - self.start_x + self.half_size) / self.resolution)
            yi = int((y_w - self.start_y + self.half_size) / self.resolution)
            line = self.bresenham(x0, y0, xi, yi)
            # free cells
            for fx, fy in line[:-1]:
                if 0 <= fx < self.size and 0 <= fy < self.size:
                    self.map[fy, fx] = 0
            # occupied
            if 0 <= xi < self.size and 0 <= yi < self.size:
                self.map[yi, xi] = 100

        # draw the trajectory on the map
        self.draw_trajectory()


    def bresenham(self, x0, y0, x1, y1):
        pts = []
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        x, y = x0, y0
        while True:
            pts.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x   += sx
            if e2 <= dx:
                err += dx
                y   += sy
        return pts


    def draw_trajectory(self):
        for px, py in self.trajectory:
            gi = int((px - self.start_x + self.half_size) / self.resolution)
            gj = int((py - self.start_y + self.half_size) / self.resolution)
            if 0 <= gi < self.size and 0 <= gj < self.size:
                self.map[gj, gi] = 50


    def publish_map(self, event):
        grid = OccupancyGrid()
        grid.header.stamp    = rospy.Time.now()
        grid.header.frame_id = 'map'
        grid.info.resolution = self.resolution
        grid.info.width      = self.size
        grid.info.height     = self.size

        # set (0,0) cell to (start_x – half, start_y – half)
        grid.info.origin.position.x = self.start_x - self.half_size
        grid.info.origin.position.y = self.start_y - self.half_size
        # identity orientation
        grid.info.origin.orientation.x = 0.0
        grid.info.origin.orientation.y = 0.0
        grid.info.origin.orientation.z = 0.0
        grid.info.origin.orientation.w = 1.0

        grid.data = self.map.flatten(order='C').tolist()
        self.map_pub.publish(grid)


if __name__ == '__main__':
    try:
        F1TENTHMapper()
    except rospy.ROSInterruptException:
        pass
