#!/usr/bin/env python3
import rospy
import numpy as np

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import tf2_ros
import tf  # for quaternion → Euler

INTENSITY_THRESHOLD = 5.0

class F1TENTHMapper:
    def __init__(self):
        rospy.init_node("f1tenth_mapper")

        # --- map parameters ---
        self.resolution = 0.05     # m per cell
        self.size       = 600      # 600×600 → 30×30 m
        self.origin     = self.size // 2
        self.map        = np.full((self.size, self.size), -1, dtype=np.int8)

        # will store the very first map←base_link pose
        self.start_x = None
        self.start_y = None

        # store trajectory in map frame
        self.trajectory = []

        # TF2 buffer & listener
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # subscriber to laser scan only
        rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=1)

        # map publisher
        self.map_pub = rospy.Publisher("/f1tenth_map", OccupancyGrid, queue_size=1)

        # publish grid at 1 Hz
        rospy.Timer(rospy.Duration(1.0), self.publish_map)

        rospy.loginfo("F1TENTH Mapper started — publishing /f1tenth_map at 1 Hz")
        rospy.spin()

    def scan_callback(self, scan: LaserScan):
        # get latest transform: map ← base_link
        try:
            xf = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                rospy.Time(0),      # latest available
                rospy.Duration(0.2))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
            return

        # extract pose
        tx = xf.transform.translation.x
        ty = xf.transform.translation.y
        q  = xf.transform.rotation
        _, _, yaw = tf.transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w]
        )

        # set initial origin on first scan
        if self.start_x is None:
            self.start_x = tx
            self.start_y = ty

        # record trajectory
        self.trajectory.append((tx, ty))

        # build angle array
        n      = len(scan.ranges)
        angles = scan.angle_min + np.arange(n) * scan.angle_increment
        ranges = np.array(scan.ranges)
        valid  = np.logical_and(np.isfinite(ranges), ranges < (scan.range_max - 1e-3))

        if len(scan.intensities) == n:
            intensities = np.array(scan.intensities)
            valid = np.logical_and(valid, intensities > INTENSITY_THRESHOLD)

        angles = angles[valid]
        ranges = ranges[valid]

        # local → map points
        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)
        c, s = np.cos(yaw), np.sin(yaw)
        R     = np.array([[c, -s], [s, c]])
        pts   = R.dot(np.vstack((xs, ys))) + np.array([[tx], [ty]])

        # origin cell in grid coords
        x0 = int((tx - self.start_x + (self.size*self.resolution)/2) /
                 self.resolution)
        y0 = int((ty - self.start_y + (self.size*self.resolution)/2) /
                 self.resolution)

        # ray-trace
        for x_w, y_w in zip(pts[0], pts[1]):
            xi = int((x_w - self.start_x + (self.size*self.resolution)/2) /
                     self.resolution)
            yi = int((y_w - self.start_y + (self.size*self.resolution)/2) /
                     self.resolution)
            line = self.bresenham(x0, y0, xi, yi)
            # free
            for fx, fy in line[:-1]:
                if 0 <= fx < self.size and 0 <= fy < self.size:
                    self.map[fy, fx] = 0
            # occupied
            if 0 <= xi < self.size and 0 <= yi < self.size:
                self.map[yi, xi] = 100

        # draw trajectory
        self.draw_trajectory()

    def bresenham(self, x0, y0, x1, y1):
        pts = []
        dx = abs(x1 - x0); dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1; sy = 1 if y0 < y1 else -1
        err = dx + dy; x, y = x0, y0
        while True:
            pts.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy; x += sx
            if e2 <= dx:
                err += dx; y += sy
        return pts

    def draw_trajectory(self):
        for px, py in self.trajectory:
            gi = int((px - self.start_x + (self.size*self.resolution)/2) /
                     self.resolution)
            gj = int((py - self.start_y + (self.size*self.resolution)/2) /
                     self.resolution)
            if 0 <= gi < self.size and 0 <= gj < self.size:
                self.map[gj, gi] = 50

    def publish_map(self, event):
        if self.start_x is None:
            return  # wait until first scan sets origin

        grid = OccupancyGrid()
        grid.header.stamp    = rospy.Time.now()
        grid.header.frame_id = "map"
        grid.info.resolution = self.resolution
        grid.info.width      = self.size
        grid.info.height     = self.size

        half = (self.size * self.resolution) / 2.0

        # center map on start_x, start_y
        grid.info.origin.position.x    = self.start_x - half
        grid.info.origin.position.y    = self.start_y - half
        grid.info.origin.orientation.z = 0.0
        grid.info.origin.orientation.w = 1.0
        grid.data = np.fliplr(np.flipud(self.map)).flatten().tolist()
        self.map_pub.publish(grid)


if __name__ == "__main__":
    try:
        F1TENTHMapper()
    except rospy.ROSInterruptException:
        pass
