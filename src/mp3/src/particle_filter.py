import numpy as np
from maze import Maze, Particle, Robot
import bisect
import rospy
from gazebo_msgs.msg import  ModelState
from gazebo_msgs.srv import GetModelState
import shutil
from std_msgs.msg import Float32MultiArray
from scipy.integrate import ode
import turtle
import matplotlib.pyplot as plt
import random
import math

def vehicle_dynamics(t, vars, vr, delta):
    curr_x = vars[0]
    curr_y = vars[1] 
    curr_theta = vars[2]
    
    dx = vr * np.cos(curr_theta)
    dy = vr * np.sin(curr_theta)
    dtheta = delta
    return [dx,dy,dtheta]

class particleFilter:
    def __init__(self, bob, world, num_particles, sensor_limit, x_start, y_start):
        self.num_particles = num_particles  # The number of particles for the particle filter
        self.sensor_limit = sensor_limit    # The sensor limit of the sensor
        particles = list()

        ##### TODO:  #####
        # Modify the initial particle distribution to be within the top-right quadrant of the world, and compare the performance with the whole map distribution.
        for i in range(num_particles):

            # (Default) The whole map
            # x = np.random.uniform(0, world.width)
            # y = np.random.uniform(0, world.height)


            ## first quadrant
            x = np.random.uniform(world.width / 2, world.width)
            y = np.random.uniform(world.height / 2, world.height)

            particles.append(Particle(x = x, y = y, maze = world, sensor_limit = sensor_limit))

        ###############

        self.particles = particles          # Randomly assign particles at the begining
        self.bob = bob                      # The estimated robot state
        self.world = world                  # The map of the maze
        self.x_start = x_start              # The starting position of the map in the gazebo simulator
        self.y_start = y_start              # The starting position of the map in the gazebo simulator
        self.modelStatePub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
        self.controlSub = rospy.Subscriber("/gem/control", Float32MultiArray, self.__controlHandler, queue_size = 1)
        self.control = []                   # A list of control signal from the vehicle
        return

    def __controlHandler(self,data):
        """
        Description:
            Subscriber callback for /gem/control. Store control input from gem controller to be used in particleMotionModel.
        """
        tmp = list(data.data)
        self.control.append(tmp)

    def getModelState(self):
        """
        Description:
            Requests the current state of the polaris model when called
        Returns:
            modelState: contains the current model state of the polaris vehicle in gazebo
        """

        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            modelState = serviceResponse(model_name='polaris')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: "+str(exc))
        return modelState

    def weight_gaussian_kernel(self,x1, x2, std = 5000):
        if x1 is None: # If the robot recieved no sensor measurement, the weights are in uniform distribution.
            return 1./len(self.particles)
        else:
            tmp1 = np.array(x1)
            tmp2 = np.array(x2)
            return np.sum(np.exp(-((tmp2-tmp1) ** 2) / (2 * std)))


    def updateWeight(self, readings_robot):
        """
        Description:
            Update the weight of each particles according to the sensor reading from the robot 
        Input:
            readings_robot: List, contains the distance between robot and wall in [front, right, rear, left] direction.
        """

        ## TODO #####
        # To assign the weights to the particles, we need to compare the similarities between the real
        # sensor measurements and the particle sensor measurements. In this MP, we recommend using a Gaussian
        # Kernel to calculate the likelihood between the two sensor readings.
        weights = []
        for i in range(self.num_particles):
            readings_particle = self.particles[i].read_sensor()
            weights.append(self.weight_gaussian_kernel(readings_robot, readings_particle))
        #print(weights)
        # Normalize the weights
        norm = np.sum(weights)
        norm_weights = weights / norm
        #assign normalized weights
        for i in range(self.num_particles):
            self.particles[i].weight = norm_weights[i]
        ###############
        # pass

    def resampleParticle(self):
        """
        Description:
            Perform resample to get a new list of particles 
        """
        particles_new = []
        weights = np.array([p.weight for p in self.particles])  # Extract weights
        cumulative_sum = np.cumsum(weights)  # Step 1: Compute cumulative sum
        num_particles = self.num_particles

        # Generate `num_particles` random numbers in [0,1] and map to cumulative sum
        random_samples = np.random.uniform(0, 1, num_particles)

        for r in random_samples:
            index = bisect.bisect_left(cumulative_sum, r)
            selected_particle = self.particles[index]
            # Create a new particle to avoid modifying references
            particles_new.append(Particle(
                x=selected_particle.x,
                y=selected_particle.y,
                heading=selected_particle.heading,
                maze=selected_particle.maze,
                sensor_limit=selected_particle.sensor_limit,
                noisy=True
            ))
            
        self.particles = particles_new


        

    def particleMotionModel(self):
        """
        Description:
            Estimate the next state for each particle according to the control input from actual robot 
            You can either use ode function or vehicle_dynamics function provided above
        """
        ## TODO #####
        
        # print(self.control)
        if len(self.control) == 0:
            return

        dt = 0.01
        for particle in self.particles:
            # Initialize state of the particle
            x, y, theta = particle.x, particle.y, particle.heading

            # Integrate through the entire control history
            for vr, delta in self.control:
                x += vr * np.cos(theta) * dt
                y += vr * np.sin(theta) * dt
                theta += delta * dt

        # Update particle state after applying all control inputs
            particle.x = x
            particle.y = y
            particle.heading = theta

        # Clear control history after applying it to all particles
        self.control.clear()
        ###############
        # pass


    def runFilter(self):
        """
        Description:
            Run PF localization
        """
        count = 0 
        time = 0.01
        h_error = []
        d_error = []
        time_set = []
        nums = []
        try:
            while True:
                ## TODO: (i) Implement Section 3.2.2. (ii) Display robot and particles on map. (iii) Compute and save position/heading error to plot. #####
                # Each time the runFilter function is run, do not forget to clear particles. Note:
                # Additionally, if you encounter lag, consider adjusting the show_frequency parameter.
                # PSUEDO CODE:
                    # sampleMotionModel(p)
                    # reading = vehicle_read_sensor()
                    # updateWeight(p, reading)
                    # p = resampleParticle(p)
                self.particleMotionModel()
                reading = self.bob.read_sensor()
                self.updateWeight(reading)
                self.resampleParticle()
                self.world.show_robot(self.bob)
                [x_estimate,y_estimate, heading_estimate] = self.world.show_estimated_location(self.particles)
                self.world.show_particles(self.particles)
                self.world.clear_objects()

                time_set.append(time)   
                time += 0.01

                heading_error = (heading_estimate - self.bob.heading) / self.bob.heading 
                distance_error = math.sqrt((self.bob.x - x_estimate)**2 + (self.bob.y - y_estimate)**2)

                h_error.append(heading_error)
                d_error.append(distance_error)
                errors = [heading_error, distance_error]

        except KeyboardInterrupt:
            print("Termination. Plotting errors...")
            # Plotting the errors
            plt.figure(figsize=(10, 5))
            plt.plot(time_set, h_error, label="Heading Error", linestyle="--", marker="o")
            plt.plot(time_set, d_error, label="Distance Error", linestyle="-", marker="x")

            plt.xlabel("Time (s)")
            plt.ylabel("Error")
            plt.title("Heading and Distance Error vs Time")
            plt.legend()
            plt.grid(True)
            plt.show()
            ###############