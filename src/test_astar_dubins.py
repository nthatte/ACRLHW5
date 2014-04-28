from astar import AStar
import numpy as np
import matplotlib.pyplot as plt
import pdb
import dubins
import math

start_state = np.array([0, 0, 0.0])
goal_state = np.array([100, 100, 0.0])
turning_radius = 2
step_size = 1

def heuristic(state1, state2):
    return np.linalg.norm(state1 - state2)

def valid_edge_function(state, primitive):
    for prim_st in primitive.path:
        st = state + rotate_state(np.asarray(prim_st),state[2])
        if np.abs(st[1]-30)<1.0 and st[0] < 80:
            return False
        if np.abs(st[1]-60)<1.0 and st[0] > 70:
            return False
        
        if st[0] < 0 or st[0] > 100 or st[1] < 0 or st[1] > 100:
            return False
    return True

def state_equality(state1, state2):
    anglecheck = (state1[2]-state2[2] + np.pi) % (2.0 * np.pi ) - np.pi
    return np.array_equal(state1[0:2],state2[0:2]) and anglecheck<0.0001

    
def rotate_state(state, angle):
    rotMatrix = np.array([[math.cos(angle), -math.sin(angle)],
                          [math.sin(angle),  math.cos(angle)]])
    rot_state = state.copy()
    rot_state[0:2] = rotMatrix.dot(state[0:2])    
    return rot_state
    

    
class motion_primitive:
    def __init__(self, delta_state, cost):
        self.delta_state = delta_state
        self.cost = cost
        path,_ = dubins.path_sample((0,0,0), self.delta_state, turning_radius, step_size)
        self.path = path

    def get_end_state(self, start_state):
        angle = start_state[2]
        rotMatrix = np.array([[math.cos(angle), -math.sin(angle)],
                              [math.sin(angle),  math.cos(angle)]])

        end_state = start_state + rotate_state(self.delta_state,start_state[2])
        return end_state

# Define primitives relative to (0,0,0)                
delta_states = [np.array([ 5,  0, 0.0]),
                np.array([ 5,  5, math.pi/2.0]),
                np.array([ 5, -5, -math.pi/2.0]),
                np.array([ 5,  5, 0.0]),
                np.array([ 5, -5, 0.0]),
                np.array([ 5,  10, math.pi/2.0]),
                np.array([ 5, -10, -math.pi/2.0]),
                np.array([ 5,  10, 0.0]),
                np.array([ 5, -10, 0.0]),
                np.array([-1,  0, 0.0])]
    
plt.figure(1)
plt.subplot(111)
motion_primitives = []
for i in range(0,len(delta_states)):
    cost = dubins.path_length((0,0,0), delta_states[i], turning_radius)
    motion_primitives.append(motion_primitive(delta_states[i],cost))
    
    pathx = []
    pathy = []
    for state in motion_primitives[i].path:
        pathx.append(state[0])
        pathy.append(state[1])
        plt.plot(pathx,pathy)

plt.show(block=False)
    
def cost_function(state, motion_primitive):
    return motion_primitive.cost # + belief cost

astar = AStar(motion_primitives, cost_function, heuristic, valid_edge_function, state_equality)
plan = astar.plan(start_state, goal_state)

if plan:
    
    pathx = [];
    pathy = [];
    for i in range(len(plan)-1,0,-1):
        seg, _ = dubins.path_sample(plan[i], plan[i-1], turning_radius, step_size)
        for state in seg:
            pathx.append(state[0])
            pathy.append(state[1])
            print state

    plt.figure(2)
    plt.subplot(111)
    plt.plot(pathx,pathy)
    plt.plot([0, 80],[30, 30])
    plt.plot([70, 100],[60, 60])

    plt.show()
