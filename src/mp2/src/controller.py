import rospy
from gazebo_msgs.srv import GetModelState, GetModelStateResponse
from gazebo_msgs.msg import ModelState
from ackermann_msgs.msg import AckermannDrive
import numpy as np
from std_msgs.msg import Float32MultiArray
import math
from util import euler_to_quaternion, quaternion_to_euler
import time

#libraries for graph plotting
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use('TkAgg')
from waypoint_list import WayPoints


class vehicleController():

    def __init__(self):
        # Publisher to publish the control input to the vehicle model
        self.controlPub = rospy.Publisher("/ackermann_cmd", AckermannDrive, queue_size = 1)
        self.prev_vel = 0
        self.L = 1.75 # Wheelbase, can be get from gem_control.py
        self.log_acceleration = True
        self.logx = []
        self.logy = []
        self.log_accel = []


    def getModelState(self):
        # Get the current state of the vehicle
        # Input: None
        # Output: ModelState, the state of the vehicle, contain the
        #   position, orientation, linear velocity, angular velocity
        #   of the vehicle
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp = serviceResponse(model_name='gem')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: "+str(exc))
            resp = GetModelStateResponse()
            resp.success = False
        return resp


    # Tasks 1: Read the documentation https://docs.ros.org/en/fuerte/api/gazebo/html/msg/ModelState.html
    #       and extract yaw, velocity, vehicle_position_x, vehicle_position_y
    # Hint: you may use the the helper function(quaternion_to_euler()) we provide to convert from quaternion to euler
    def extract_vehicle_info(self, currentPose):

        ####################### TODO: Your TASK 1 code starts Here #######################
        pos_x, pos_y, vel, yaw = 0, 0, 0, 0
        
        pos_x = currentPose.pose.position.x
        pos_y = currentPose.pose.position.y

        linear_vel = currentPose.twist.linear
        vel = math.sqrt(linear_vel.x**2 + linear_vel.y**2 + linear_vel.z**2)

        x, y, z, w = currentPose.pose.orientation.x, currentPose.pose.orientation.y, currentPose.pose.orientation.z, currentPose.pose.orientation.w
        roll, pitch, yaw =  quaternion_to_euler(x, y, z, w)


        ####################### TODO: Your Task 1 code ends Here #######################

        return pos_x, pos_y, vel, yaw # note that yaw is in radian

    # Task 2: Longtitudal Controller
    # Based on all unreached waypoints, and your current vehicle state, decide your velocity
    def longititudal_controller(self, curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints):

        ####################### TODO: Your TASK 2 code starts Here #######################
        
        max_vel = 21
        min_vel = 15
        target_velocity = min_vel
        
        curvature = 0
        target_point = None
        min_dist = 12 

        for i, wp in enumerate(future_unreached_waypoints):
            dist = math.sqrt((wp[0] - curr_x)**2 + (wp[1] - curr_y)**2)
            if dist <= min_dist:
                min_dist = dist
                target_point = wp

        if target_point is None:
            target_point = future_unreached_waypoints[-1]
            min_dist = math.sqrt((target_point[0] - curr_x)**2 + (target_point[1] - curr_y)**2)

        if min_dist > 0:    
            curvature = 2 * (target_point[0] - curr_x) / (min_dist**2) #https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf
        
        if abs(curvature) <= 0.08:
            target_velocity = max_vel # for less curvatures
        elif abs(curvature) > 0.08:
            target_velocity = min_vel 

        if target_velocity > max_vel:
            target_velocity = max_vel
        elif target_velocity < min_vel:
            target_velocity = min_vel

        print(f"Target velocity: {target_velocity:.2f} m/s | Curvature: {curvature:.4f} | Min_Dist: {min_dist:.4f}") 
        ####################### TODO: Your TASK 2 code ends Here #######################
        return target_velocity


    # Task 3: Lateral Controller (Pure Pursuit)
    def pure_pursuit_lateral_controller(self, curr_x, curr_y, curr_yaw, target_point, future_unreached_waypoints):
        
        ####################### TODO: Your TASK 3 code starts Here #######################
        target_x, target_y = target_point

        min_ld = 2
        max_ld = 5 
        curv_fact = 0.8 #sensitivity for exponential decay
        lookahead_point = None

        if(len(future_unreached_waypoints) >= 2):
            next_wp = future_unreached_waypoints[1]
            curvature = abs(math.atan2(next_wp[1] - target_point[1], next_wp[0] - target_point[0]) - curr_yaw)

            # applied exponential decay for smoother dynamic estimation of lookahead distance
            # inspired from https://www.mdpi.com/2079-9292/10/22/2812 

            lookahead_distance = min_ld + (max_ld - min_ld) * math.exp(-curv_fact * curvature) 

            #find the first waypoint in the list at least ld away from the curr_pos

            for i in range(len(future_unreached_waypoints)-1):
                p1 = np.array(future_unreached_waypoints[i])
                p2 = np.array(future_unreached_waypoints[i + 1])

                if np.linalg.norm(p2 - np.array([curr_x,curr_y])) > lookahead_distance:
                    lookahead_point = p2
                    break

        if lookahead_point is None:
            lookahead_point = future_unreached_waypoints[-1]

        dx = lookahead_point[0] - curr_x
        dy = lookahead_point[1] - curr_y
        ld = np.sqrt((dx)**2 + (dy)**2)
        ld = max(min_ld, ld)

        target_angle = np.arctan2(dy, dx)
        alpha = target_angle - curr_yaw

        target_steering = np.arctan(2*self.L*np.sin(alpha) / ld)
        print('target angle: ', target_steering)
        print('ld_steer: ', ld)
                
        ####################### TODO: Your TASK 3 code starts Here #######################
        return target_steering


    def execute(self, currentPose, target_point, future_unreached_waypoints):
        # Compute the control input to the vehicle according to the
        # current and reference pose of the vehicle
        # Input:
        #   currentPose: ModelState, the current state of the vehicle
        #   target_point: [target_x, target_y]
        #   future_unreached_waypoints: a list of future waypoints[[target_x, target_y]]
        # Output: None

        curr_x, curr_y, curr_vel, curr_yaw = self.extract_vehicle_info(currentPose)
        # Start should be [x,y,z] = [0.015203, -98.006679, 0.105925]
        # [roll, pitch, yaw] = [0.000002, 0.000082, -0.009020]

        # print('cur info, x: ', curr_x, 'y: ', curr_y, 'vel: ', curr_vel, 'yaw: ', curr_yaw)
        #for plotting
        self.logx.append(curr_x)
        self.logy.append(curr_y)

        # Acceleration Profile
        if self.log_acceleration:
            acceleration = (curr_vel- self.prev_vel) * 100 # Since we are running in 100Hz
            #for plotting
            if(acceleration >= 5):
                print(acceleration)
            self.log_accel.append(acceleration)

        self.prev_vel = curr_vel

        target_velocity = self.longititudal_controller(curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints)
        target_steering = self.pure_pursuit_lateral_controller(curr_x, curr_y, curr_yaw, target_point, future_unreached_waypoints)


        #Pack computed velocity and steering angle into Ackermann command
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = target_velocity
        newAckermannCmd.steering_angle = target_steering

        # Publish the computed control input to vehicle model
        self.controlPub.publish(newAckermannCmd)

    def stop(self):
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = 0
        self.controlPub.publish(newAckermannCmd)

        #Plotting Graphs
        # Plot graph for Problem 7
        waypoints = WayPoints()
        pos_list = waypoints.getWayPoints()
        plt.plot(self.logx, self.logy, label="Trajectory")
        plt.plot([x[0] for x in pos_list], [x[1] for x in pos_list], '*', label="Waypoints")
        plt.plot([0],[-98], '*', label="Start Point", color="red")
        plt.legend()
        plt.show()
        
        # Plot graph for Problem 5
        plt.plot(self.log_accel)
        plt.axhline(y=5, color="red", linestyle='--', label="Comfort Threshold")
        plt.axhline(y=-5, color="red", linestyle='--')
        plt.xlabel("Distance (m)")
        plt.ylabel("Acceleration (m/s^2)")    
        plt.legend()
        plt.show()

