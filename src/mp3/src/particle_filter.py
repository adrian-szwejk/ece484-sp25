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
        sum_of_particle_weight = 0
        # print("Reading robot is ", readings_robot.deserialize('orientation'))
        for particle in self.particles:
            particle_sensor_measurement = particle.read_sensor()
            particle.weight = self.weight_gaussian_kernel(readings_robot, particle_sensor_measurement)
            sum_of_particle_weight += particle.weight
        # Normalize the weight of the particles so that the sum of the weight equals to 1
        for particle in self.particles:
            particle.weight = particle.weight / sum_of_particle_weight
        ###############
        # pass

    def resampleParticle(self):
        """
        Description:
            Perform resample to get a new list of particles 
        """
        cumulative_sum_of_weight = []
        for i in range(self.num_particles):
            s = 0
            if len(cumulative_sum_of_weight) != 0:
                s = cumulative_sum_of_weight[-1]
            s += self.particles[i].weight
            cumulative_sum_of_weight.append(s)
        particles_new = list()
        for _ in range(self.num_particles):
            ## TODO #####
            # 1. Calculate an array of the cumulative sum of the weights.
            # 2. Randomly generate a number and determine which range in that cumulative weight array to which
            # the number belongs.
            num = random.random()
            particle_index = bisect.bisect_left(cumulative_sum_of_weight, num)
            # 3. The index of that range would correspond to the particle that should be created.
            # 4. Repeat sampling until you have the desired number of samples
            current_particle = self.particles[particle_index]
            particle = Particle(
                        x = current_particle.x, 
                        y = current_particle.y, 
                        maze = current_particle.maze, 
                        heading = current_particle.heading, 
                        weight = 1, 
                        sensor_limit = current_particle.sensor_limit, 
                        noisy = True)
            particles_new.append(particle)
            ###############
        self.particles = particles_new
        

    def particleMotionModel(self):
        """
        Description:
            Estimate the next state for each particle according to the control input from actual robot 
            You can either use ode function or vehicle_dynamics function provided above
        """
        ## TODO #####
        
        # print(self.control)
        for control_input in self.control:# Normalize the particle weight
            # sum_of_particle_weight = 0
            # for particle in self.particles:
            #     sum_of_particle_weight += particle.weight
            # for particle in self.particles:
            #     particle.weight = particle.weight / sum_of_particle_weight
            v = control_input[0]
            delta = control_input[1]
            for particle in self.particles:
                x, y, heading = particle.x, particle.y, particle.heading 
                dx, dy, dtheta = vehicle_dynamics(0, [x, y, heading], v, delta)
                particle.x = x + 0.01 * dx
                particle.y = y + 0.01 * dy
                particle.heading += 0.01 * dtheta
        self.control = []
        ###############
        # pass


    def runFilter(self):
        """
        Description:
            Run PF localization
        """
        count = 0 
        h_error = []
        d_error = []
        nums = []
        while count < 10000:
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

            heading_error = (heading_estimate - self.bob.heading) / self.bob.heading 
            distance_error = math.sqrt((self.bob.x - x_estimate)**2 + (self.bob.y - y_estimate)**2)

            h_error.append(heading_error)
            d_error.append(distance_error)
            count += 1
            nums.append(count)
        
        plt.plot(h_error, nums)
        plt.plot(d_error, nums)
        plt.show()
            ###############