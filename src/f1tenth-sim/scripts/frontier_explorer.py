#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from tf import TransformListener

class FrontierExplorer:
    def __init__(self):
        rospy.init_node('frontier_explorer')

        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.goal_pub = rospy.Publisher('/exploration_goal', PoseStamped, queue_size=1)
        self.tf_listener = TransformListener()

        self.map_data = None
        self.resolution = None
        self.origin = None
        self.width = None
        self.height = None

        rospy.loginfo("✅ Frontier Explorer started.")
        rospy.spin()

    def map_callback(self, msg):
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin = msg.info.origin
        self.map_data = np.array(msg.data).reshape((self.height, self.width))

        try:
            (trans, rot) = self.tf_listener.lookupTransform('map', 'car_1_base_link', rospy.Time(0))
            robot_pos = (trans[0], trans[1])
        except:
            rospy.logwarn("⚠️ TF lookup failed.")
            return

        frontiers = self.find_frontiers()
        if not frontiers:
            rospy.loginfo("❌ No frontiers found.")
            return

        best_cell = self.select_closest_frontier(robot_pos, frontiers)
        goal = self.grid_to_pose(best_cell)
        self.goal_pub.publish(goal)
        rospy.loginfo(f"🚀 Published goal at {goal.pose.position.x:.2f}, {goal.pose.position.y:.2f}")

    def find_frontiers(self):
        frontiers = []
        for y in range(1, self.map_data.shape[0] - 1):
            for x in range(1, self.map_data.shape[1] - 1):
                if self.map_data[y][x] == 0:
                    neighbors = self.map_data[y-1:y+2, x-1:x+2].flatten()
                    if -1 in neighbors:
                        frontiers.append((x, y))
        return frontiers

    def select_closest_frontier(self, robot_pos, frontiers):
        min_dist = float('inf')
        best = None
        for (x, y) in frontiers:
            wx = x * self.resolution + self.origin.position.x
            wy = y * self.resolution + self.origin.position.y
            dist = np.linalg.norm(np.array([wx, wy]) - np.array(robot_pos))
            if dist < min_dist:
                min_dist = dist
                best = (x, y)
        return best

    def grid_to_pose(self, cell):
        x, y = cell
        wx = x * self.resolution + self.origin.position.x
        wy = y * self.resolution + self.origin.position.y
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = wx
        pose.pose.position.y = wy
        pose.pose.orientation.w = 1.0
        return pose

if __name__ == '__main__':
    try:
        FrontierExplorer()
    except rospy.ROSInterruptException:
        pass
