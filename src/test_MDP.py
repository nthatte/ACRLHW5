from MDP import MDP
import numpy as np
import matplotlib.pyplot as plt
import pdb

#define states
goal_state = np.array([49, 49])
(X,Y) = np.meshgrid(range(50), range(50), indexing='xy')
states = np.array(zip(X.flatten(), Y.flatten()))

actions = [np.array([ 1,  0]),
           np.array([ 1,  1]),
           np.array([ 0,  1]),
           np.array([-1,  1]),
           np.array([-1,  0]),
           np.array([-1, -1]),
           np.array([ 0, -1]),
           np.array([ 1, -1])]

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
V, pi = mdp.value_iteration()

value_mat = np.zeros((50,50))
Sx = []
Sy = []
Ax = []
Ay = []
for s in states:
    value_mat[s[0], s[1]] = V[str(s)]
    Sx.append(s[0])
    Sy.append(s[1])
    Ax.append(pi[str(s)][0])
    Ay.append(pi[str(s)][1])

plt.pcolor(value_mat.T)
plt.quiver(Sx, Sy, Ax, Ay)
plt.plot([5, 49],[4, 4], linewidth = 4)
plt.plot([4, 4],[5, 49], linewidth = 4)
plt.show()
