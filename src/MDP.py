import copy
import pdb
import matplotlib.pyplot as plt
import numpy as np

class MDP:
    def __init__(self,state_list,valid_actions_function,cost_function,gamma = 1, converge_thr = 1):
        self.states = state_list
        self.actions = valid_actions_function
        self.cost = cost_function
        self.gamma = gamma
        self.converge_thr = converge_thr

    "Solve an MDP by value iteration"
    def value_iteration(self, policy = None, value = None, num_iter = 100, plot = False, world_size = 0):
        #initialize policy and value
        if value:
            policy = {}
        elif policy:
            value = self.policy_evaluation(policy, num_iter = num_iter, plot = plot, world_size = world_size)
        else:
            value = dict([(s.tostring(),0) for s in self.states])
            policy = {} 

        if plot:
            plt.ion()
            fig = plt.figure(3)

        i = 0
        total_value_old = 0
        total_value = float('inf')
        #run to convergence
        while abs(total_value - total_value_old) > self.converge_thr:
            if i > num_iter:
                break
            total_value_old = total_value
            total_value = 0
            print i

            if plot:
                value_mat = np.zeros((world_size,world_size))
                Sx = []
                Sy = []
                Ax = []
                Ay = []

            for s in self.states:
                string_s = s.tostring()

                #argmax value to get best action
                value_a_min = (400, np.array([0, 0]))
                actions_list = self.actions(s)
                for a in actions_list:
                    value_temp = self.cost(s,a) + value[(s + a).tostring()]
                    if  value_temp <= value_a_min[0]:
                        value_a_min = (value_temp, a)

                #bellman update
                value[string_s] = value_a_min[0]
                policy[string_s] = value_a_min[1]
                total_value += value[string_s]

                if plot:
                    value_mat[s[0]-1, s[1]-1] = value[string_s]
                    Sx.append(s[0]-0.5)
                    Sy.append(s[1]-0.5)
                    Ax.append(policy[string_s][0])
                    Ay.append(policy[string_s][1])

            i += 1

        if plot:
            fig.clear()
            plt.pcolor(value_mat.T)
            plt.quiver(Sx, Sy, Ax, Ay)
            plt.show()
            plt.pause(0.00001)
        return value

    def policy_evaluation(self, policy, init_value = None, num_iter = 100, plot = False, world_size = 0):
        if init_value:
            value = init_value
        else:
            value = dict([(s.tostring(),0) for s in self.states])

        if plot:
            plt.ion()
            fig = plt.figure(1)

        i = 0
        total_value_old = 0
        total_value = float('inf')
        while abs(total_value - total_value_old) > self.converge_thr:
            if i > num_iter:
                break
            total_value_old = total_value
            total_value = 0
            i += 1
            print i

            if plot:
                value_mat = np.zeros((world_size,world_size))
                Sx = []
                Sy = []
                Ax = []
                Ay = []

            for s in self.states:
                string_s = s.tostring()
                a = policy[string_s]
                if (s+a).tostring() in value:
                    vp = value[(s+a).tostring()]
                else:
                    vp = 100000
                value[string_s] = self.cost(s,a) + self.gamma*vp
                total_value += value[string_s]

                if plot:
                    value_mat[s[0] -1, s[1] -1] = value[string_s]
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

        return value
