from astar import AStar
import numpy as np
import matplotlib.pyplot as plt
import pdb
import dubins
import math


def heuristic(state1, state2):
    return np.linalg.norm(state1 - state2)

def valid_edge_function(state, primitive):
    for prim_st in primitive.path:
        st = state + rotate_state(np.asarray(prim_st),state[2])

    return True

def wrapToPi(angle):
    return (angle + np.pi) % (2.0 * np.pi ) - np.pi
    
def state_equality(state1, state2):
    anglecheck = wrapToPi(state1[2]-state2[2])
    return np.array_equal(state1[0:2],state2[0:2]) and anglecheck<0.0001

    
def rotate_state(state, angle):
    rotMatrix = np.array([[math.cos(angle), -math.sin(angle)],
                          [math.sin(angle),  math.cos(angle)]])
    rot_state = state.copy()
    rot_state[0:2] = rotMatrix.dot(state[0:2])    
    return rot_state
    
    
class motion_primitive:
    turning_radius = 2
    step_size = .1
    def __init__(self, delta_state, cost):
        self.delta_state = delta_state
        self.cost = cost
        path,_ = dubins.path_sample((0,0,0), self.delta_state, self.turning_radius, self.step_size)
        self.path = path

    def get_end_state(self, start_state):
        angle = start_state[2]
        rotMatrix = np.array([[math.cos(angle), -math.sin(angle)],
                              [math.sin(angle),  math.cos(angle)]])

        end_state = start_state + rotate_state(self.delta_state,start_state[2])
        return end_state

        
def cost_function(state, motion_primitive):
    return motion_primitive.cost # + belief cost

def get_xytheta_paths(plan):
    turning_radius = 2
    step_size = 1
    pathx = []
    pathy = []
    paththeta = []
    path = []
    for i in range(len(plan)-1,0,-1):
        seg, _ = dubins.path_sample(plan[i], plan[i-1], turning_radius, step_size)
        path += seg
    return path
