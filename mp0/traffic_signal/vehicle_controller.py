from enum import Enum, auto
import copy
from typing import List

class TLMode(Enum):
    GREEN=auto()
    YELLOW=auto()
    RED=auto()

class VehicleMode(Enum):
    Normal = auto()
    Brake = auto()
    Accel = auto()
    HardBrake = auto()

class State:
    x: float 
    y: float 
    theta: float 
    v: float 
    agent_mode: VehicleMode 

    def __init__(self, x, y, theta, v, agent_mode: VehicleMode):
        pass 

def decisionLogic(ego: State, other: State):
    output = copy.deepcopy(ego)

    # TODO: Edit this part of decision logic
    
    # * d = 300m, din = 20m, dout = 15m
    # * Entrance [d − din, d − dout]
        # The car cannot be in this region if the light is red.
    # * Exit [d − dout, d]
        # The car cannot be slow in this region if the light is red.

    # * Green Light: The car can drive normally.
    if other.signal_mode == TLMode.GREEN:
        if ego.agent_mode != VehicleMode.Accel:
            output.agent_mode = VehicleMode.Accel
    

    # * Yellow Light/Red Light
    if other.signal_mode == TLMode.YELLOW or other.signal_mode == TLMode.RED:
        # Far from enterance
        if ego.agent_mode != VehicleMode.Brake and ego.agent_mode != VehicleMode.HardBrake and other.x-70 < ego.x < other.x-50 and ego.v > 0:
            output.agent_mode = VehicleMode.Brake
        # Close to enterance
        if ego.agent_mode != VehicleMode.HardBrake and other.x-50 < ego.x <= other.x-20 and ego.v > 0:
            output.agent_mode = VehicleMode.HardBrake

    

    ###########################################
    # DO NOT CHANGE THIS
    assert not (other.signal_mode == TLMode.RED and (ego.x>other.x-20 and ego.x<other.x-15)), "Run Red Light"  
    assert not (other.signal_mode == TLMode.RED and (ego.x>other.x-15 and ego.x<other.x) and ego.v<1), "Stop at Intersection"

    return output 