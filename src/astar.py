#from heapq import heappush, heappop # for priority queue
from pqdict import PQDict
import time
import pdb
import numpy as np
from astar_fcns import wrapToPi
import copy

class node:
    def __init__(self, state, parent, path_from_parent, g, h):
        self.state = state
        self.g = g
        self.h = h
        self.f = g + h
        self.parent = parent
        self.path = path_from_parent

    def __cmp__(self, other):
        return cmp(self.f, other.f)

class AStar:
    def __init__(self, motion_primitives, cost_function, heuristic_function, valid_edge_function, state_equality_function):

        self.motion_primitives = motion_primitives
        self.cost = cost_function
        self.heuristic = heuristic_function
        self.valid_edge = valid_edge_function
        self.state_is_equal = state_equality_function

    def getChildren(self, cur_node):
        children = []
        cur_angle = np.around(wrapToPi(cur_node.state[2])/(np.pi/2.0))
        print cur_node.state[2], wrapToPi(cur_node.state[2]), cur_angle

        for primitive in self.motion_primitives[cur_angle]:
            if self.valid_edge(cur_node.state, primitive):
                new_state = primitive.get_end_state(cur_node.state) #cur_node.state + primitive.delta_state
                g = cur_node.g + self.cost(cur_node.state, primitive)
                h = self.heuristic(new_state, self.goal_state) 
                child = node(new_state, cur_node, cur_node.state + primitive.path, g, h)
                children.append(child)
                
        return children

    def reconstruct_path(self,cur_node):
        print 'Reconstructing path:'
        path = cur_node.path[::-1]
        while (cur_node.parent):
            print cur_node.state
            if cur_node.parent.path is not None:
                path = np.append(path, cur_node.parent.path[::-1], axis=0)
            cur_node = cur_node.parent
        return path[::-1]

    def plan(self, start_state, goal_state):
        #PQ = pqdict()
        V = {}

        self.goal_state = goal_state
        h0 = self.heuristic(start_state, goal_state)
        n0 = node(start_state, None, None, 0, h0)

        key0 = n0.state.tostring()
        PQ = PQDict(key0=n0)

        i = 0 
        while PQ:
            current = PQ.popitem()[1]
            print current.state[2]
            pdb.set_trace()
            if(self.state_is_equal(current.state, goal_state)):
                return (self.reconstruct_path(current), current.f)

            V[current.state.tostring()] = copy.deepcopy(current)

            #get children

            children = self.getChildren(current)#do set parent, should return an array of nodes
            
            for child in children:
                i += 1
                if i%100 == 0:
                    print 'iteration '+str(i)
                
                child_key = child.state.tostring()
                if child_key in V:
                    continue
                
                if (child_key in PQ):
                    existing_child = PQ[child_key]
                    if(child.g >= existing_child.g):
                        continue
                    else:
                        PQ.updateitem(child_key, child)
                else:
                    #print child.state, current.state
                    #pdb.set_trace()
                    PQ.additem(child_key,child)
        
        print 'A* Failed'
        return None   
