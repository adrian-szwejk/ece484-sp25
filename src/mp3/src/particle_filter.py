import numpy as np
from maze import Maze, Particle, Robot
import bisect
import rospy
from gazebo_msgs.msg import  ModelState
from gazebo_msgs.srv import GetModelState
import shutil
from std_msgs.msg import Float32MultiArray
from scipy.integrate import ode

import random

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
            # x = np.random.uniform(0, world.width / 2)
            # y = np.random.uniform(0, world.height / 2)


            ## first quadrant
            x = np.random.uniform(0, world.width / 2)
            y = np.random.uniform(world.height / 2, world.height)

            particles.append(Particle(x = -x, y = y, maze = world, sensor_limit = sensor_limit))

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
        for particle in self.particles:
            particle_sensor_measurement = particles.read_sensor()
            particle.weight = self.weight_gaussian_kernel(particle_sensor_measurement, reading_robot)
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
        num_of_samples = self.num_particles // 2
        t = 0
        while t < num_of_samples:
            particles_new = list()
            ## TODO #####
            # 1. Calculate an array of the cumulative sum of the weights.
            cumulative_sum_of_weight = []
            for i in range(self.num_particles):
                s = 0
                if len(cumulative_sum_of_weight) != 0:
                    s = cumulative_sum_of_weight[-1]
                s += particles[i].weight
                cumulative_sum_of_weight.append(s)
            # 2. Randomly generate a number and determine which range in that cumulative weight array to which
            # the number belongs.
            num = random.random()
            paricle_index = 0
            while cumulative_sum_of_weight[particle_index] < num and particle_index < len(cumulative_sum_of_weight):
                particle_index += 1
            # 3. The index of that range would correspond to the particle that should be created.
            # 4. Repeat sampling until you have the desired number of samples
            for i in range(self.num_particles):
                if i != particle_index:
                    particles_new.append(self.particles[i]) 
                else:
                    current_particle = self.particle[i]
                    particle = Particle(x = current_particle.x, y = current_particle.y, maze = current_particle.maze, heading = current_particle.heading, weight = current_particle.weight, sensor_limit = current_particle.sensor_limit, noisy = True)
            ###############
            self.particles = particles_new
            # Normalize the particle weight
            sum_of_particle_weight = 0
            for particle in self.particles:
                sum_of_particle_weight += particle.weight
            for particle in self.particles:
                particle.weight = particle.weight / sum_of_particle_weight
            t += 1

    def particleMotionModel(self):
        """
        Description:
            Estimate the next state for each particle according to the control input from actual robot 
            You can either use ode function or vehicle_dynamics function provided above
        """
        ## TODO #####
        # print(self.control)

        ###############
        # pass


    def runFilter(self):
        """
        Description:
            Run PF localization
        """
        count = 0 
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
            reading = self.getModelState()
            self.updateWeight(reading)
            self.resampleParticle()
            count += 1
            ###############
