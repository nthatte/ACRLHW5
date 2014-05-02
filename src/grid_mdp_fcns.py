import numpy as np

class GridWorldMDP:
    def __init__(self, map_array, goal_state, start_state, bridge_probabilities, bridge_locations, radius = 3.0, replan_factor = 1.0):
       
        self.map_array = map_array
        world_size = self.map_array.shape[0]
        self.goal_state = goal_state

        #get valid states (x,y) locations on the map 
        (x,y) = np.meshgrid(np.arange(1,world_size+1),np.arange(1,world_size+1))
        ind = np.where(map_array!=0)
        self.states = np.array(zip(x[ind], y[ind]))

        #list of actions on grid
        self.actions = [np.array([ 1,  0]),
                        np.array([-1,  0]),
                        np.array([ 0,  1]),
                        np.array([ 0, -1]),
                        np.array([ 1,  1]),
                        np.array([-1, -1]),
                        np.array([ 1, -1]),
                        np.array([-1,  1])]

        self.radius = radius
        self.bridge_locations = bridge_locations
        if len(bridge_locations.shape) == 1:
            self.bridge_probabilities = [bridge_probabilities]
        else:
            self.bridge_probabilities = bridge_probabilities

        if len(self.bridge_locations.shape) > 1:
            self.replan_costs = [replan_factor*np.linalg.norm(start_state - bridge_location) 
                for bridge_location in bridge_locations.T]
        else:
            self.replan_costs = [replan_factor*np.linalg.norm(start_state - bridge_locations)]

    def __valid_state(self, state):
        if self.map_array[state[1] -1, state[0] - 1] != 0.0:
           return True
        return False

    def valid_actions_function(self, state):
        if np.array_equal(state,self.goal_state):
            return [np.array([ 0, 0])]
        else:
            valid_actions = []
            for (i, a) in enumerate(self.actions):
                state_p = state + a
                if self.__valid_state(state_p):

                    #check if a state above and below is valid
                    if (self.__valid_state(state_p + np.array([0, 1]))
                        or self.__valid_state(state_p + np.array([0, -1]))):

                        #check if a state left and right is valid
                        if (self.__valid_state(state_p + np.array([1, 0]))
                            or self.__valid_state(state_p + np.array([-1, 0]))):

                            # check diagonal right/up violation
                            if i == 4 and (self.__valid_state(state + np.array([0, 1]))
                                or self.__valid_state(state + np.array([1, 0]))):
                                valid_actions.append(a)

                            # check diagonal left/down violation
                            elif i == 5 and (self.__valid_state(state + np.array([0, -1]))
                                or self.__valid_state(state + np.array([-1, 0]))):
                                valid_actions.append(a)
                            
                            # check diagonal right/down violation
                            elif i == 6 and (self.__valid_state(state + np.array([0, -1]))
                                or self.__valid_state(state + np.array([1, 0]))):
                                valid_actions.append(a)

                            # check diagonal left/up violation
                            elif i == 7 and (self.__valid_state(state + np.array([0, 1]))
                                or self.__valid_state(state + np.array([-1, 0]))):
                                valid_actions.append(a)

                            #add straight actions
                            elif i== 0 or i==1 or  i==2  or i==3:
                                valid_actions.append(a)
            return valid_actions

    '''
    def cost_function(self, state, action):
        return np.sqrt(action[0]**2 + action[1]**2)
    '''

    def cost_function(self, state, action):
        action_cost = np.linalg.norm(action)
        #for obs in bridge_locations:
        if len(self.bridge_locations.shape) > 1:
            dists_to_bridge = self.bridge_locations.T-state
            dists_to_bridge = np.sqrt(dists_to_bridge[:,0]**2 + dists_to_bridge[:,1]**2)
        else:
            dists_to_bridge = state - bridge_locations
            dists_to_bridge = [np.sqrt(dists_to_bridge[0]**2 + dists_to_bridge[1]**2)]

        prob_open = 1.0
        replan_cost = 0.0
        for (i, dist_to_bridge) in enumerate(dists_to_bridge):
            if dist_to_bridge <= self.radius:
                prob_open *= self.bridge_probabilities[i]
                if self.replan_costs[i] > replan_cost:
                    replan_cost = self.replan_costs[i]
            i += 1
        return action_cost + (1.0-prob_open)*replan_cost
