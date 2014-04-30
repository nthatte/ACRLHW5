from heapq import heappush, heappop # for priority queue
import time
import pdb

class node:
    def __init__(self, state, parent, g, h):
        self.state = state
        self.g = g
        self.h = h
        self.f = g + h
        self.parent = parent
    def __cmp__(self, other):
        return cmp(self.f, other.f)

class AStar:
    def __init__(self, motion_primitives, cost_function, heuristic_function, valid_edge_function, state_equality_function):
        self.PQ = []
        self.PQ_hash = {}
        self.V = {}

        self.motion_primitives = motion_primitives
        self.cost = cost_function
        self.heuristic = heuristic_function
        self.valid_edge = valid_edge_function
        self.state_is_equal = state_equality_function

    def getChildren(self, cur_node):
        children = []
        cur_angle = cur_node.state[2]
        for primitive in self.motion_primitives[cur_angle]:
            if self.valid_edge(cur_node.state, primitive,True):
                new_state = primitive.get_end_state(cur_node.state) #cur_node.state + primitive.delta_state
                g = cur_node.g + self.cost(cur_node.state, primitive)
                h = self.heuristic(new_state, self.goal_state) 
                child = node(new_state, cur_node, g, h)
                children.append(child)
                
        return children

    def reconstruct_path(self,curr_node):
        path = [curr_node.state]
        while (curr_node.parent):
            path.append(curr_node.parent.state)
            curr_node = curr_node.parent
        return path

    def plan(self, start_state, goal_state):
        self.goal_state = goal_state
        h0 = self.heuristic(start_state, goal_state)
        n0 = node(start_state, None, 0, h0)

        heappush(self.PQ,n0)
        self.PQ_hash[n0.state.tostring()] = n0

        while self.PQ:
            current = heappop(self.PQ)
            
            del self.PQ_hash[current.state.tostring()]
            if(self.state_is_equal(current.state, goal_state)):
                return self.reconstruct_path(current), current.f

            self.V[current.state.tostring()] = current

            #get children

            children = self.getChildren(current)#do set parent, should return an array of nodes
            
            for child in children:
                if child.state.tostring() in self.V:
                    continue
                
                if (child.state.tostring() in self.PQ_hash):
                    existing_child = self.PQ_hash.get(child.state.tostring())
                    if(child.g >= existing_child.g):
                        continue
                    else:
                        existing_child = child
                else:
                    heappush(self.PQ,child)
                    self.PQ_hash[child.state.tostring()] = child
        
        print 'A* Failed'
        return None   
