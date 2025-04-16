#!/usr/bin/env python3

import rospy
import numpy as np
import heapq
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from tf import TransformListener

class AStarPlanner:
    def __init__(self):
        rospy.init_node('astar_planner')

        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.goal_sub = rospy.Subscriber('/exploration_goal', PoseStamped, self.goal_callback)
        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=1)

        self.tf_listener = TransformListener()
        self.map = None
        self.map_info = None

        rospy.loginfo("A* planner initialized.")
        rospy.spin()

    def map_callback(self, msg):
        self.map = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info

    def goal_callback(self, goal_msg):
        if self.map is None:
            rospy.logwarn("Map not received yet.")
            return

        try:
            (trans, rot) = self.tf_listener.lookupTransform('map', 'car_1_base_link', rospy.Time(0))
            start_world = (trans[0], trans[1])
        except:
            rospy.logwarn("TF failed to get robot pose.")
            return

        start = self.world_to_grid(start_world)
        goal = self.world_to_grid((goal_msg.pose.position.x, goal_msg.pose.position.y))

        rospy.loginfo(f"Planning from {start} to {goal}")
        path = self.astar(start, goal)

        if path:
            self.publish_path(path)
        else:
            rospy.logwarn("No path found!")

    def world_to_grid(self, pos):
        x = int((pos[0] - self.map_info.origin.position.x) / self.map_info.resolution)
        y = int((pos[1] - self.map_info.origin.position.y) / self.map_info.resolution)
        return (x, y)

    def grid_to_world(self, cell):
        x = cell[0] * self.map_info.resolution + self.map_info.origin.position.x
        y = cell[1] * self.map_info.resolution + self.map_info.origin.position.y
        return (x, y)

    def is_valid(self, cell):
        x, y = cell
        if x < 0 or y < 0 or x >= self.map.shape[1] or y >= self.map.shape[0]:
            return False
        return self.map[y][x] == 0  # Free space only

    def neighbors(self, cell):
        x, y = cell
        steps = [(-1,0),(1,0),(0,-1),(0,1)]  # 4-connectivity
        return [(x+dx, y+dy) for dx,dy in steps if self.is_valid((x+dx, y+dy))]

    def heuristic(self, a, b):
        return np.linalg.norm(np.array(a) - np.array(b))

    def astar(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (0 + self.heuristic(start, goal), 0, start, []))
        visited = set()

        while open_set:
            est_total, cost, current, path = heapq.heappop(open_set)
            if current in visited:
                continue
            visited.add(current)

            new_path = path + [current]

            if current == goal:
                return new_path

            for neighbor in self.neighbors(current):
                if neighbor not in visited:
                    heapq.heappush(open_set, (
                        cost + 1 + self.heuristic(neighbor, goal),
                        cost + 1,
                        neighbor,
                        new_path
                    ))
        return None

    def publish_path(self, grid_path):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()

        for cell in grid_path:
            x, y = self.grid_to_world(cell)
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        rospy.loginfo(f"Published path with {len(path_msg.poses)} points.")

if __name__ == '__main__':
    try:
        AStarPlanner()
    except rospy.ROSInterruptException:
        pass
