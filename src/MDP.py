import copy
import pdb
import matplotlib.pyplot as plt
import numpy as np

class MDP:
    def __init__(self,state_list,valid_actions_function,cost_function,gamma = 1):
        self.states = state_list
        self.actions = valid_actions_function
        self.cost = cost_function
        self.gamma = gamma

    "Solve an MDP by value iteration"
    def value_iteration(self, policy = None, value = None, plot = False, world_size = 0):
        if policy:
            value = self.policy_evaluation(policy)
        else:
            if value == None:
                value = dict([(str(s),0) for s in self.states])
            policy = {} 

        value_old = dict([(str(s),float('inf')) for s in self.states])
        #run to convergence
        i = 0

        if plot:
            world_size = 10
            plt.ion()
            fig = plt.figure(1)

        while value != value_old:
            print i
            value_old = copy.deepcopy(value)

            if plot:
                value_mat = np.zeros((world_size,world_size))
                Sx = []
                Sy = []
                Ax = []
                Ay = []

            for s in self.states:
                #argmax value to get best action
                value_a_tuples = [(self.cost(s,a) + value[str(s + a)], a) for a in
                    self.actions(s)]
                
                value_a_min = min(value_a_tuples, key = lambda value_a: value_a[0])

                #bellman update
                value[str(s)] = value_a_min[0]
                policy[str(s)] = value_a_min[1]

                value_mat[s[0], s[1]] = value[str(s)]
                Sx.append(s[0])
                Sy.append(s[1])
                Ax.append(policy[str(s)][0])
                Ay.append(policy[str(s)][1])

            if plot:
                fig.clear()
                plt.pcolor(value_mat.T)
                plt.quiver(Sx, Sy, Ax, Ay)
                plt.plot([5, world_size - 1],[4, 4], linewidth = 4)
                plt.plot([4, 4],[5, world_size - 1], linewidth = 4)
                plt.show()
                plt.pause(0.00001)
            i += 1
        return (value, policy)

    def policy_evaluation(self, policy, init_value = None):
        if init_value:
            value = init_value
        else:
            value = dict([(str(s),0) for s in self.states])

        value_old = dict([(str(s),float('inf')) for s in self.states])
        while value != value_old:
            value_old = copy.deepcopy(value)
            for s in self.states:
                a = policy[str(s)]
                if str(s+a) in value:
                    vp = value[str(s+a)]
                else:
                    vp = 100000
                value[str(s)] = self.cost(s,a) + self.gamma*vp

        return value
