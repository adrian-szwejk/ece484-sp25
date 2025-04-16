from abc import ABC, abstractmethod
import copy
from itertools import count
import numpy as np
global_index = count()
class AbstractState(ABC):
    def __init__(self, state, goal, dist_from_start=0, use_heuristic=True):
        self.state = state
        self.goal = goal
        self.tiebreak_idx = next(global_index)
        self.dist_from_start = dist_from_start
        self.use_heuristic = use_heuristic
        if use_heuristic:
            self.h = self.compute_heuristic()
        else:
            self.h = 0
    @abstractmethod
    def get_neighbors(self):
        pass
    
    @abstractmethod
    def is_goal(self):
        pass

    @abstractmethod
    def compute_heuristic(self):
        pass
    
    @abstractmethod
    def __lt__(self, other):
        if self.tiebreak_idx < other.tiebreak_idx:
            return True
    @abstractmethod
    def __hash__(self):
        pass
    @abstractmethod
    def __eq__(self, other):
        pass

class F1_tenth(AbstractState):
    def __init__(self, state, goal, dist_from_start, use_heuristic, time_interval):
        '''
        state: the current coordinate (x, y) of the F1_tenth Car
        goal: the destination coordinate (x, y) of the F1_tenth Car
        dist_from_start: integer
        use_heuristic: boolean
        '''
        # NOTE: AbstractState constructor does not take cost_per_letter
        super().__init__(state, goal, dist_from_start, use_heuristic)
        self.time_interval = time_interval
    def get_neighbors(self):
        '''
        TODO: This function will return the set of possible coordinates given the
        time interval and the control signal that is send from LIDAR.
        '''
        pass
    def is_goal(self):
        estimation = 0.1
        return np.sqrt((self.state[0] - self.goal[0])**2 + (self.state[1] - self.goal[1])**2) < estimation
    def __lt__(self, other):    
        # You should return True if the current state has a lower g + h value than "other"
        # If they have the same value then you should use tiebreak_idx to decide which is smaller
        if self.dist_from_start + self.h < other.dist_from_start + other.h:
            return True
        elif self.dist_from_start + self.h > other.dist_from_start + other.h:
            return False
        return self.tiebreak_idx < other.tiebreak_idx
    def compute_heuristic(self):
        # Manhattan distance between two points
        a = self.state
        b = self.goal
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    def __hash__(self):
        return hash(self.state)
    def __eq__(self, other):
        return self.state == other.state
    def __str__(self):
        return self.state
    def __repr__(self):
        return self.state