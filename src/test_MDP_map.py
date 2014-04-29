from MDP import MDP
import numpy as np
import matplotlib.pyplot as plt
import scipy.io as sio
import pdb

#define states
world_size = 50

filename = 'map_2.mat'
map_struct_packed = sio.loadmat(filename, squeeze_me = True)['map_struct'].item()
map_array = map_struct_packed[4][2].T
goal_state  = np.array(map_struct_packed[6].item())

#get valid states (x,y) locations on the map 
(y,x) = np.meshgrid(np.arange(0,world_size),np.arange(0,world_size))
ind = np.where(map_array==1.0)
states = np.array(zip(x[ind], y[ind]))

actions = [np.array([ 1,  0]),
           np.array([ 1,  1]),
           np.array([ 0,  1]),
           np.array([-1,  1]),
           np.array([-1,  0]),
           np.array([-1, -1]),
           np.array([ 0, -1]),
           np.array([ 1, -1])]

init_value = {}
for s in states:
    init_value[s.tostring()] = np.linalg.norm(s - goal_state)
        
def valid_state(state):
    if state[0] >= 0 and state[0] < world_size:
        if state[1] >= 0 and state[1] < world_size:
            if map_array[state[0],state[1]] != 1.0:
                return False
            if state[0] >= 1 and state[0] < (world_size - 1):
                if (map_array[state[0]-1,state[1]] != 1.0 
                    and map_array[state[0]+1,state[1]] != 1.0):
                        return False
            if state[1] >= 1 and state[1] < (world_size - 1):
                if (map_array[state[0],state[1]-1] != 1.0
                    and map_array[state[0],state[1]+1] != 1.0):
                        return False
            return True
    return False

def valid_actions_function(state):
    if np.array_equal(state,goal_state):
        return [np.array([ 0, 0])]
    else:
        return [a for a in actions if valid_state(state + a)]

def cost_function(state, action):
    return np.sqrt(action[0]**2 + action[1]**2)
    #return np.linalg.norm(action)

mdp = MDP(states, valid_actions_function, cost_function, converge_thr = 1, gamma = 1)
#V, pi = mdp.value_iteration(policy = init_policy, plot = True, world_size = world_size)
#V, pi = mdp.value_iteration(policy = init_policy, value = init_value, plot = True, world_size = world_size)
#V, pi = mdp.value_iteration(policy = init_policy)
#V, pi = mdp.value_iteration(policy = init_policy, value = init_value)
#V, pi = mdp.value_iteration(value = init_value, plot = True, world_size = world_size)
V, pi = mdp.value_iteration(value = init_value)
#V, pi = mdp.value_iteration(plot = True, world_size = world_size)
#V, pi = mdp.value_iteration()

value_mat = np.zeros((world_size,world_size))
Sx = []
Sy = []
Ax = []
Ay = []
for s in states:
    value_mat[s[0], s[1]] = V[s.tostring()]
    Sx.append(s[0])
    Sy.append(s[1])
    Ax.append(pi[s.tostring()][0])
    Ay.append(pi[s.tostring()][1])

plt.ioff()
fig = plt.figure(1)
plt.pcolor(value_mat.T)
plt.quiver(Sx, Sy, Ax, Ay)
plt.show()
