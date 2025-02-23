from enum import Enum, auto
import copy
from typing import List

class PedestrianMode(Enum):
    Normal=auto()

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


# * ONLY USE: if statements
    # • arithmetic operators (+,-,/,*)
    # • logical operators (and, or , not)
    # • comparison operators (<=, <, ==, ! =, >, >=)
    # • functions
    # assertions (for defining safety)
    # • "any" keyword
    # • "all" keyword

# * DON'T USE:
    # • For loops
    # • While loops
    # • Print
    # • Numpy
    # • Else/elif
    # • Exponent/Root

# * output.agent_mode: Normal, Brake, HardBrake, Accel
def decisionLogic(ego: State, other: State):
    output = copy.deepcopy(ego)

    # TODO: Edit this part of decision logic
    
    # Stop accelerating
    if ego.agent_mode == VehicleMode.Accel and other.dist < 6:
        output.agent_mode = VehicleMode.Normal

    # Braking
    if ego.agent_mode == VehicleMode.Normal and other.dist < 4:
        output.agent_mode = VehicleMode.Brake
    if ego.agent_mode == VehicleMode.Brake and other.dist < 3:
        output.agent_mode = VehicleMode.HardBrake
    
    # Accel after stopping 
    if (ego.agent_mode == VehicleMode.Brake or ego.agent_mode == VehicleMode.HardBrake) and other.dist > 6:
        output.agent_mode = VehicleMode.Accel
    if (ego.agent_mode == VehicleMode.Brake or ego.agent_mode == VehicleMode.HardBrake) and other.dist > 4:
        output.agent_mode = VehicleMode.Normal
    ###########################################

    # DO NOT CHANGE THIS
    assert other.dist > 2.0, "Too Close!"

    return output 



