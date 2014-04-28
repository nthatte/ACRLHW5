from MDP import MDP
import numpy as np
import matplotlib.pyplot as plt
import pdb

#define states
world_size = 50
goal_state = np.array([world_size - 1, world_size - 1])
(X,Y) = np.meshgrid(range(world_size), range(world_size))
states = np.array(zip(X.flatten(), Y.flatten()))

actions = [np.array([ 1,  0]),
           np.array([ 1,  1]),
           np.array([ 0,  1]),
           np.array([-1,  1]),
           np.array([-1,  0]),
           np.array([-1, -1]),
           np.array([ 0, -1]),
           np.array([ 1, -1])]

init_policy = {}
for s in states:
    angle = np.degrees(np.arctan2(goal_state[1] - s[1], goal_state[0] - s[0]))
    if angle > -22.5 and angle <= 22.5:
        init_policy[s.tostring()] = actions[0]
    elif angle > 22.5 and angle <= 67.5:
        init_policy[s.tostring()] = actions[1]
    elif angle > 67.5 and angle <= 112.5:
        init_policy[s.tostring()] = actions[2]
    elif angle > 112.5 and angle <= 157.5:
        init_policy[s.tostring()] = actions[3]
    elif (angle > 157.5 and angle <= 180) or (angle <= -157.5 and angle >= -180):
        init_policy[s.tostring()] = actions[4]
    elif angle > -157.5 and angle <= -112.5:
        init_policy[s.tostring()] = actions[4]
    elif angle > -112.5 and angle <= -67.5:
        init_policy[s.tostring()] = actions[6]
    elif angle > --67.5 and angle <= -22.5:
        init_policy[s.tostring()] = actions[7]
    
        
def valid_state(state):
    if state[0] >= 0 and state[0] <= goal_state[0]:
        if state[1] >=0 and state[1] <= goal_state[1]:
            if state[1] == 4 and state[0] > 4:
                return False
            if state[0] == 4 and state[1] > 4:
                return False
            return True
    return False

def valid_actions_function(state):
    if np.array_equal(state,goal_state):
        return [np.array([ 0, 0])]
    else:
        return [a for a in actions if valid_state(state + a)]

def cost_function(state, action):
    return np.linalg.norm(action)

mdp = MDP(states, valid_actions_function, cost_function)
#V, pi = mdp.value_iteration(policy = init_policy, plot = True, world_size = world_size)
V, pi = mdp.value_iteration(policy = init_policy)
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

plt.pcolor(value_mat.T)
plt.quiver(Sx, Sy, Ax, Ay)
plt.plot([5, world_size - 1],[4, 4], linewidth = 4)
plt.plot([4, 4],[5, world_size - 1], linewidth = 4)
plt.show()
