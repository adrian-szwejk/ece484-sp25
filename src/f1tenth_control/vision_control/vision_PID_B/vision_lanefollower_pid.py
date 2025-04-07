import rospy
import numpy as np
import argparse

# from gazebo_msgs.msg import  ModelState
from controller import vehicleController
import time
from util import euler_to_quaternion, quaternion_to_euler

def run_model():
    rospy.init_node("model_dynamics")
    controller = vehicleController()


    def shutdown():
        """Stop the car when this ROS node shuts down"""
        controller.stop()
        rospy.loginfo("Stop the car")

    rospy.on_shutdown(shutdown)

    rate = rospy.Rate(50)  # 100 Hz
    rospy.sleep(0.0)
    

    while not rospy.is_shutdown():
        rate.sleep()  # Wait a while before trying to get a new state
        # imagepoints = controller.getImagepoints()
        # worldpoints = controller.getWorldpoints()
        # worldwaypoints = controller.Convert_to_worldcoords(waypoints,imagepoints,worldpoints)
        # print(f"waypoints = {waypoints}")
        # print(f"worldwaypoints = {worldwaypoints}")
        controller.execute()
        pass


if __name__ == "__main__":
    try:
        run_model()
        
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Shutting down")
