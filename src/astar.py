from heapq import heappush, heappop # for priority queue
import time
import pdb

class node:
    def __init__(self, state, h, parent = None, g=0):
        self.state = state
        self.g = g
        self.h = h
        self.f = g + h
        self.parent = parent
    def __cmp__(self, other):
        return cmp(self.f, other.f)

class AStar:
    def __init__(start_state, goal_state, world_map, motion_primitives, heuristic, valid_edge_function):
        self.PQ = []
        self.PQ_hash = {}
        self.V = {}

        self.start_state = start_state
        self.goal_state = goal_state
        self.world_map = world_map
        self.motion_primitives = motion_primitives
        self.heuristic = heuristic
        self.valid_edge = valid_edge_function

    def getChildren(self, cur_node):
        children = []
        for primitive in self.motion_primitives:
            if self.valid_edge(primitive,self.world_map):
                g = cur_node.g + motion_primitive.cost
                child = node(cur_node.state + motion_primitive.delta_state, cur_node, g)
                children.append(child)
                
        return children

    def atGoal(self, cur_node):
        return self.goal_state.is_close(cur_node.state)
    
    def reconstruct_path(self,curr_node):
        path = [curr_node]
        while (curr_node.parent):
            path.append(curr_node.parent)
            curr_node = curr_node.parent
        return path

    def plan(self):
        h0 = self.heuristic(self.start_state, self.goal_state)
        n0 = node(self.start_state, h0)

        heappush(self.PQ,n0)
        self.PQ_hash[n0.state] = n0

        while self.PQ:
            current = heappop(self.PQ)
            
            del self.PQ_hash[current.state]
            if(self.atGoal(current, self.goal)):
                return self.reconstruct_path(current)

            self.V[current.state] = current

            #get children

            children = self.getChildren(current)#do set parent, should return an array of nodes
            
            for child in children:
                if child.state in self.V:
                    continue
                
                if (child.state in self.PQ_hash):
                    existing_child = self.PQ_hash.get(child.state)
                    if(child.g >= existing_child.g):
                        continue
                    else:
                        existing_child = child
                else:
                    heappush(self.PQ,child)
                    self.PQ_hash[child.state] = child
        
        print 'A* Failed'
        return None   
