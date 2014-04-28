import copy

class MDP:
    def __init__(self,state_list,valid_actions_function,cost_function,gamma = 1):
        self.states = state_list
        self.actions = valid_actions_function
        self.cost = cost_function
        self.gamma = gamma

    "Solve an MDP by value iteration"
    def value_iteration(self, policy = None, value = None):
        if policy:
            value = self.policy_evaluation(init_policy)
        else:
            if value == None:
                value = dict([(str(s),0) for s in self.states])
            policy = {} 

        value_old = dict([(str(s),float('inf')) for s in self.states])

        #run to convergence
        i = 0
        while i < 3:
            print i
            value_old = copy.deepcopy(value)

            for s in self.states:
                #argmax value to get best action
                value_a_tuples = [(self.cost(s,a) + value[str(s + a)], a) for a in
                    self.actions(s)]
                
                value_a_min = min(value_a_tuples, key = lambda value_a: value_a[0])

                #bellman update
                value[str(s)] = value_a_min[0]
                policy[str(s)] = value_a_min[1]

            i += 1

        return (value, policy)

    def policy_evaluation(self, policy, init_value = None):
        if init_value:
            value = init_value
        else:
            value = dict([(str(s),0) for s in self.states])

        value_old = dict([(str(s),float('inf')) for s in self.states])
        while value != value_old:
            value_old = value
            for s in self.states:
                a = policy(s)
                value[str(s)] = self.cost(s,a) + self.gamma*value[str(s+a)]

        return value
