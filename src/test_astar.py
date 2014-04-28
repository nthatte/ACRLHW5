from astar import AStar
import numpy as np
import matplotlib.pyplot as plt
import pdb

start_state = np.array([0, 0])
goal_state = np.array([10, 10])

def heuristic(state1, state2):
    return np.linalg.norm(state1 - state2)

def valid_edge_function(state, primitive):
    if state[1] == 3 and state[0] < 8:
        return False
    if state[1] == 6 and state[0] > 2:
        return False
    return True

def state_equality(state1, state2):
    return np.array_equal(state1,state2)

class motion_primitive:
    def __init__(self, delta_state):
        self.delta_state = delta_state

    def get_end_state(self, state):
        return state + self.delta_state 

delta_states = [np.array([ 1,  0]),
                np.array([ 1,  1]),
                np.array([ 0,  1]),
                np.array([-1,  1]),
                np.array([-1,  0]),
                np.array([-1, -1]),
                np.array([ 0, -1]),
                np.array([ 1, -1])]
    
motion_primitives = [motion_primitive(delta_state) for delta_state in delta_states]

def cost_function(state, motion_primitive):
    return np.linalg.norm(motion_primitive.delta_state)

astar = AStar(motion_primitives, cost_function, heuristic, valid_edge_function, state_equality)
plan = astar.plan(start_state, goal_state)

pathx = [state[0] for state in plan]
pathy = [state[1] for state in plan]

plt.plot(pathx,pathy)
plt.plot([0, 7],[3, 3])
plt.plot([3, 10],[6, 6])

plt.show()
