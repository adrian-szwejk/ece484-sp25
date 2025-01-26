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
    if ego.agent_mode != VehicleMode.Normal and other.signal_mode == TLMode.GREEN:
        output.agent_mode = VehicleMode.Normal

    # * Yellow Light: If car is in the entrance region, it speed up to exit region, o.w. slow down
    if other.signal_mode == TLMode.YELLOW:
        # In Entrance [d − din, d − dout]
        if ego.agent_mode != VehicleMode.Accel and ego.x > other.x-20 and ego.x < other.x-15:
            output.agent_mode = VehicleMode.Accel
        # In Exit [d − dout, d]
        if ego.x > other.x-15 and ego.x < other.x:
            if ego.v < 1:
                output.agent_mode = VehicleMode.Normal
        # Far from enterance
        if ego.agent_mode != VehicleMode.Brake and ego.x > other.x-25 and ego.v > 1:
            output.agent_mode = VehicleMode.Brake
        # Close to enterance
        # if ego.agent_mode == VehicleMode.Brake and ego.x > other.x-22 and ego.v > 1:
        #     output.agent_mode = VehicleMode.HardBrake

    # * Red Light: If in exit region speed up, o.w. slow down/stop
    if other.signal_mode == TLMode.RED:
        # In Exit [d − dout, d]
        if ego.agent_mode != VehicleMode.Accel and  ego.x > other.x-15 and ego.x < other.x:
            output.agent_mode = VehicleMode.Accel
        # Far from enterance
        if ego.agent_mode != VehicleMode.Brake and ego.x > other.x-25 and ego.v > 1:
            output.agent_mode = VehicleMode.Brake
        # Close to enterance
        # if ego.agent_mode == VehicleMode.Brake and ego.x > other.x-22 and ego.v > 1:
        #     output.agent_mode = VehicleMode.HardBrake

    

    ###########################################
    # DO NOT CHANGE THIS
    assert not (other.signal_mode == TLMode.RED and (ego.x>other.x-20 and ego.x<other.x-15)), "Run Red Light"  
    assert not (other.signal_mode == TLMode.RED and (ego.x>other.x-15 and ego.x<other.x) and ego.v<1), "Stop at Intersection"

    return output 