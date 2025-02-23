import rospy
from gazebo_msgs.srv import GetModelState, GetModelStateResponse
from gazebo_msgs.msg import ModelState
from ackermann_msgs.msg import AckermannDrive
import numpy as np
from std_msgs.msg import Float32MultiArray
import math
from util import euler_to_quaternion, quaternion_to_euler
import time

class vehicleController():

    def __init__(self):
        # Publisher to publish the control input to the vehicle model
        self.controlPub = rospy.Publisher("/ackermann_cmd", AckermannDrive, queue_size = 1)
        self.prev_vel = 0
        self.L = 1.75 # Wheelbase, can be get from gem_control.py
        self.log_acceleration = False

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
        
        # currentPose -> ModelState {model_name : string, pose : geometry_msgs/Pose, twist : geometry_msgs/Twist, reference_frame : string}
            # model_name = model to set state (pose/twist)
            # pose = Desired pose in reference frame
                # position : Point
                    # x : float64
                    # y : float64
                    # z : float64
                # orientation : Quaternion
                    # x : float64
                    # y : float64
                    # z : float64
                    # w : float64
            # twist = Desired twist in reference frame
                # linear : Vector3
                    # x : float64
                    # y : float64
                    # z : float64
                # angular : Vector3
                    # x : float64
                    # y : float64
                    # z : float64
            # reference_frame = set pose/twist relative to frame of this entity
                # Leaving reference_frame = "word"/"map"/empty -> defaults to world-frame
        pos_x = currentPose.pose.position.x
        pos_y = currentPose.pose.position.y

        linear_vel = currentPose.twist.linear
        vel = math.sqrt(linear_vel.x**2 + linear_vel.y**2 + linear_vel.z**2)

        # [roll, pitch, yaw] = quaternion_to_euler(x, y, z, w)
        x, y, z, w = currentPose.pose.orientation.x, currentPose.pose.orientation.y, currentPose.pose.orientation.z, currentPose.pose.orientation.w
        roll, pitch, yaw =  quaternion_to_euler(x, y, z, w)


        ####################### TODO: Your Task 1 code ends Here #######################

        return pos_x, pos_y, vel, yaw # note that yaw is in radian

    # Task 2: Longtitudal Controller
    # Based on all unreached waypoints, and your current vehicle state, decide your velocity
    def longititudal_controller(self, curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints):

        ####################### TODO: Your TASK 2 code starts Here #######################
        target_velocity = 10


        ####################### TODO: Your TASK 2 code ends Here #######################
        return target_velocity


    # Task 3: Lateral Controller (Pure Pursuit)
    def pure_pursuit_lateral_controller(self, curr_x, curr_y, curr_yaw, target_point, future_unreached_waypoints):
        
        ####################### TODO: Your TASK 3 code starts Here #######################
        # target_point = [target_x, target_y]
        # future_enreached_waypoints = [[target_x, target_y], ...] ???
        target_x, target_y = target_point

        # Paremeters to tune
        Kdd = 0.1
        min_ld = 5
        max_ld = 20
        # min_ld = 3, max_ld = 15
            # Works well until about 30th waypoint? When starts to swing side to side a bit (waypoints not in a straight line??)
            # Then fails at 47th waypoint b/c just turns into grass???
        # min_ld = 5, max_ld = 20
            #  Smoothed out issue at 30th waypoint but on some curves was turning a little late
        # Reached all the waypoints: 134.35

        # target is look-ahead distance away from vehicle
        # np.clip(Kdd * speed, min_ld, max_ld)
        dx = target_x - curr_x
        dy = target_y - curr_y
        ld = np.sqrt((dx)**2 + (dy)**2)
        print('ld', ld)
        ld = np.clip(ld, min_ld, max_ld)

        # alpha = arctan2(target_y, target_x) - yaw
        target_angle = np.arctan2(dy, dx)
        alpha = target_angle - curr_yaw

        # δ = arctan(2*L*sin(α) / ld)
            # L = wheel base
            # δ = target angle
            # α = Angle between line of current direction to line toward TP
            # ld = Distance vehicle -> TP

        target_steering = np.arctan(2*self.L*np.sin(alpha) / ld)
        print('target angle: ', target_steering)
        
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

        # Acceleration Profile
        if self.log_acceleration:
            acceleration = (curr_vel- self.prev_vel) * 100 # Since we are running in 100Hz



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
