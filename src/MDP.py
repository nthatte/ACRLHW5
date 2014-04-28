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
                value = dict([(s.tostring(),0) for s in self.states])
            policy = {} 

        value_old = dict([(s.tostring(),float('inf')) for s in self.states])
        #run to convergence
        i = 0

        if plot:
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
                string_s = s.tostring()

                #argmax value to get best action
                value_a_min = (float('inf'), None)
                actions_list = self.actions(s)
                for a in actions_list:
                    value_temp = self.cost(s,a) + value[(s + a).tostring()]
                    if  value_temp <= value_a_min[0]:
                        value_a_min = (value_temp, a)

                #bellman update
                value[string_s] = value_a_min[0]
                policy[string_s] = value_a_min[1]

                if plot:
                    value_mat[s[0], s[1]] = value[string_s]
                    Sx.append(s[0])
                    Sy.append(s[1])
                    Ax.append(policy[string_s][0])
                    Ay.append(policy[string_s][1])

            if plot:
                fig.clear()
                plt.pcolor(value_mat.T)
                plt.quiver(Sx, Sy, Ax, Ay)
                plt.show()
                plt.pause(0.00001)
            i += 1
        return (value, policy)

    def policy_evaluation(self, policy, init_value = None):
        if init_value:
            value = init_value
        else:
            value = dict([(s.tostring(),0) for s in self.states])

        value_old = dict([(s.tostring(),float('inf')) for s in self.states])
        while value != value_old:
            value_old = copy.deepcopy(value)
            for s in self.states:
                a = policy[s.tostring()]
                if (s+a).tostring() in value:
                    vp = value[(s+a).tostring()]
                else:
                    vp = 100000
                value[s.tostring()] = self.cost(s,a) + self.gamma*vp

        return value
