from MDP import MDP
import numpy as np
import matplotlib.pyplot as plt
import scipy.io as sio
import pdb
import cPickle as pickle

#define states
world_size = 50

map_name = 'map_2'
map_struct_packed = sio.loadmat(map_name + '.mat', squeeze_me = True)['map_struct'].item()
map_array = map_struct_packed[3]
goal_state  = np.array(map_struct_packed[6].item())

#get valid states (x,y) locations on the map 
(x,y) = np.meshgrid(np.arange(1,world_size+1),np.arange(1,world_size+1))
ind = np.where(map_array!=0)
states_list = zip(x[ind], y[ind])
states = np.array(states_list)

actions = [np.array([ 1,  0]),
           np.array([-1,  0]),
           np.array([ 0,  1]),
           np.array([ 0, -1]),
           np.array([ 1,  1]),
           np.array([-1, -1]),
           np.array([ 1, -1]),
           np.array([-1,  1])]

init_value = {}
for s in states:
    init_value[s.tostring()] = np.linalg.norm(s - goal_state)
        
def valid_state(state):
    if map_array[state[1] -1, state[0] - 1] != 0.0:
       return True
#   if tuple(state) in states_list:
#      return True
#    if state[0] >= 0 and state[0] < world_size:
#        if state[1] >= 0 and state[1] < world_size:
#            if map_array[state[0],state[1]] != 1.0:
#                return False
#            if state[0] >= 1 and state[0] < (world_size - 1):
#                if (map_array[state[0]-1,state[1]] != 1.0 
#                    and map_array[state[0]+1,state[1]] != 1.0):
#                        return False
#            if state[1] >= 1 and state[1] < (world_size - 1):
#                if (map_array[state[0],state[1]-1] != 1.0
#                    and map_array[state[0],state[1]+1] != 1.0):
#                        return False
    return False

def valid_actions_function(state):
    if np.array_equal(state,goal_state):
        return [np.array([ 0, 0])]
    else:
        valid_actions = []
        for (i, a) in enumerate(actions):
            state_p = state + a
            if valid_state(state_p):

                #check if a state above and below is valid
                if (valid_state(state_p + np.array([0, 1]))
                    or valid_state(state_p + np.array([0, -1]))):

                    #check if a state left and right is valid
                    if (valid_state(state_p + np.array([1, 0]))
                        or valid_state(state_p + np.array([-1, 0]))):

                        # check diagonal right/up violation
                        if i == 4 and (valid_state(state + np.array([0, 1]))
                            or valid_state(state + np.array([1, 0]))):
                            valid_actions.append(a)

                        # check diagonal left/down violation
                        elif i == 5 and (valid_state(state + np.array([0, -1]))
                            or valid_state(state + np.array([-1, 0]))):
                            valid_actions.append(a)
                        
                        # check diagonal right/down violation
                        elif i == 6 and (valid_state(state + np.array([0, -1]))
                            or valid_state(state + np.array([1, 0]))):
                            valid_actions.append(a)

                        # check diagonal left/up violation
                        elif i == 7 and (valid_state(state + np.array([0, 1]))
                            or valid_state(state + np.array([-1, 0]))):
                            valid_actions.append(a)

                        #add straight actions
                        elif i== 0 or i==1 or  i==2  or i==3:
                            valid_actions.append(a)
        return valid_actions
                

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

with open(map_name +'value.pickle', 'wb') as handle:
    pickle.dump(V, handle)

value_mat = np.zeros((world_size,world_size))
Sx = []
Sy = []
Ax = []
Ay = []
for s in states:
    value_mat[s[0]-1, s[1]-1] = V[s.tostring()]
    Sx.append(s[0]-0.5)
    Sy.append(s[1]-0.5)
    Ax.append(pi[s.tostring()][0])
    Ay.append(pi[s.tostring()][1])

plt.ioff()
fig = plt.figure(1)
plt.pcolor(value_mat.T)
plt.quiver(Sx, Sy, Ax, Ay)
plt.show()
