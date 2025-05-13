#!/usr/bin/env python3
import rospy
import numpy as np
import open3d as o3d
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped

INTENSITY_THRESHOLD = 5.0

class F1TENTHMapper:
    def __init__(self):
        rospy.init_node("f1tenth_mapper_icp")

        # --- map parameters ---
        self.resolution = 0.003  # m per cell
        self.size = 2048         # 600×600 → 30×30 m
        self.map = np.full((self.size, self.size), -1, dtype=np.int8)

        # --- ICP-based pose estimation state ---
        self.prev_pcd = None
        self.tx = 0.0
        self.ty = 0.0
        self.yaw = 0.0

        # --- origin for grid centering ---
        self.start_x = None
        self.start_y = None

        # store trajectory in map frame
        self.trajectory = []

        # subscriber to laser scan only
        rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=1)

        # map publisher
        self.map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1)
        # trajectory publisher
        self.traj_pub = rospy.Publisher("/trajectory", Path, queue_size=1)

        # publish grid at 1 Hz
        rospy.Timer(rospy.Duration(1.0), self.publish_map)

        rospy.loginfo("F1TENTH Mapper started — publishing /map and /trajectory at 1 Hz")
        rospy.spin()

    def scan_callback(self, scan: LaserScan):
        # build angle & range arrays, apply intensity filter
        n = len(scan.ranges)
        angles = scan.angle_min + np.arange(n) * scan.angle_increment
        ranges = np.array(scan.ranges)
        valid = np.logical_and(np.isfinite(ranges), ranges < (scan.range_max - 1e-3))
        if len(scan.intensities) == n:
            intensities = np.array(scan.intensities)
            valid = np.logical_and(valid, intensities > INTENSITY_THRESHOLD)
        angles = angles[valid]
        ranges = ranges[valid]

        # local points in robot frame
        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)

        # create current scan point cloud
        pts3d = np.vstack((xs, ys, np.zeros_like(xs))).T
        pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts3d))

        # perform ICP against previous scan
        if self.prev_pcd is not None:
            reg = o3d.pipelines.registration.registration_icp(
                pcd,
                self.prev_pcd,
                max_correspondence_distance=0.5,
                estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint())
            T = reg.transformation
            dx, dy = T[0,3], T[1,3]
            dtheta = math.atan2(T[1,0], T[0,0])
            # accumulate into global pose
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            self.tx  += c*dx - s*dy
            self.ty  += s*dx + c*dy
            self.yaw += dtheta

        # store for next ICP
        self.prev_pcd = pcd

        # set initial origin on first callback
        if self.start_x is None:
            self.start_x = self.tx
            self.start_y = self.ty

        # record trajectory
        self.trajectory.append((self.tx, self.ty))

        # publish trajectory as Path
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "map"
        for x, y in self.trajectory:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.traj_pub.publish(path)

        # transform points to map frame
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        R = np.array([[c, -s], [s, c]])
        pts_map = R.dot(np.vstack((xs, ys))) + np.array([[self.tx], [self.ty]])

        # grid centering offset
        half = (self.size * self.resolution) / 2.0

        # origin cell index
        x0 = int((self.tx - self.start_x + half) / self.resolution)
        y0 = int((self.ty - self.start_y + half) / self.resolution)

        # ray-trace each endpoint
        for x_w, y_w in zip(pts_map[0], pts_map[1]):
            xi = int((x_w - self.start_x + half) / self.resolution)
            yi = int((y_w - self.start_y + half) / self.resolution)
            line = self.bresenham(x0, y0, xi, yi)
            # free along the ray
            for fx, fy in line[:-1]:
                if 0 <= fx < self.size and 0 <= fy < self.size:
                    self.map[fy, fx] = 0
            # occupied at endpoint
            if 0 <= xi < self.size and 0 <= yi < self.size:
                self.map[yi, xi] = 100

        # overlay trajectory on map
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
                err += dy; x += sx
            if e2 <= dx:
                err += dx; y += sy
        return pts

    def draw_trajectory(self):
        half = (self.size * self.resolution) / 2.0
        for px, py in self.trajectory:
            gi = int((px - self.start_x + half) / self.resolution)
            gj = int((py - self.start_y + half) / self.resolution)
            if 0 <= gi < self.size and 0 <= gj < self.size:
                self.map[gj, gi] = 50

    def publish_map(self, event):
        if self.start_x is None:
            return
        grid = OccupancyGrid()
        grid.header.stamp = rospy.Time.now()
        grid.header.frame_id = "map"
        grid.info.resolution = self.resolution
        grid.info.width = self.size
        grid.info.height = self.size
        half = (self.size * self.resolution) / 2.0
        grid.info.origin.position.x = self.start_x - half
        grid.info.origin.position.y = self.start_y - half
        grid.info.origin.orientation.w = 1.0
        grid.data = np.fliplr(np.flipud(self.map)).flatten().tolist()
        self.map_pub.publish(grid)

if __name__ == "__main__":
    try:
        F1TENTHMapper()
    except rospy.ROSInterruptException:
        pass
