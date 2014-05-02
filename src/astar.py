#from heapq import heappush, heappop # for priority queue
from pqdict import PQDict
import time
import pdb
import numpy as np
from astar_fcns import wrapToPi, motion_primitive
import copy


class node:
    def __init__(self, state, parent, path_from_parent, g, h, isbackward = False):
        self.state = state
        self.g = g
        self.h = h
        self.f = g + h
        self.parent = parent
        self.path = path_from_parent
        self.isbackward = isbackward

    def __cmp__(self, other):
        return cmp(self.f, other.f)

class AStar:
    def __init__(self, motion_primitives, cost_function, heuristic_function, valid_edge_function, state_equality_function, plot = False):

        self.motion_primitives = motion_primitives
        self.cost = cost_function
        self.heuristic = heuristic_function
        self.valid_edge = valid_edge_function
        self.state_is_equal = state_equality_function
        self.plot = plot

    @profile
    def getChildren(self, cur_node):
        children = []
        cur_angle = np.around(wrapToPi(cur_node.state[2])/motion_primitive.theta_res)
        #print cur_node.state[2], wrapToPi(cur_node.state[2]), cur_angle

        for primitive in self.motion_primitives[cur_angle]:
            if self.valid_edge(cur_node.state, primitive, self.plot):
                #if (cur_node.isbackward and not primitive.isbackward) or not cur_node.isbackward:
                new_state = primitive.get_end_state(cur_node.state) #cur_node.state + primitive.delta_state
                g = cur_node.g + self.cost(cur_node.state, primitive)
                h = self.heuristic(new_state, self.goal_state) 
                offset = np.array((cur_node.state[0], cur_node.state[1], 0.0))
                child = node(new_state, cur_node, offset + primitive.path, g, h, primitive.isbackward)
                #print child.path[0]
                #if cur_node.path != None:
                #    print cur_node.path[-1]
                #pdb.set_trace()
                children.append(child)
            #pdb.set_trace()
                
        return children

    def reconstruct_path(self,cur_node):
        print 'Reconstructing path:'
        path = cur_node.path[::-1]
        indices = [0]
        while (cur_node.parent):
            #print cur_node.state
            if cur_node.parent.path is not None:
                path = np.append(path, cur_node.parent.path[::-1], axis=0)
                cur_length = indices[-1]
                indices.append(len(cur_node.parent.path) + cur_length)
            cur_node = cur_node.parent
        total_length = len(path)
        indices = [total_length - i for i in indices]
        return path[::-1], indices

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
            #print '\n'
            #print current.state[2]
            if(self.state_is_equal(current.path, goal_state)):
                path, indices = self.reconstruct_path(current)
                return (path, indices, current.f)

            V[current.state.tostring()] = copy.deepcopy(current)

            #get children
            children = self.getChildren(current)#do set parent, should return an array of nodes
            
            for child in children:
                i += 1
                if i%100 == 0:
                    print 'A* iteration '+str(i)
                
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
                    #if(child.state[2] < 0):
                    #    pdb.set_trace()
                    PQ.additem(child_key,child)
        
        print 'A* Failed'
        return None   
