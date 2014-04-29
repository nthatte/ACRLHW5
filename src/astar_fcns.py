import numpy as np
import matplotlib.pyplot as plt
import pdb
import dubins
import math
from shapely.geometry import LineString
from shapely.ops import cascaded_union
from shapely.geometry import Point

def wrapToPi(angle):
    return (angle + np.pi) % (2.0 * np.pi ) - np.pi

def rotate_state(state, angle):
    rotMatrix = np.array([[np.cos(angle), -np.sin(angle)],
                        [np.sin(angle),  np.cos(angle)]])
    rot_state = state.copy()
    rot_state[0:2] = rotMatrix.dot(state[0:2])    
    return rot_state

class motion_primitive:
    turning_radius = 2
    step_size = 0.1
    def __init__(self, delta_state):
        self.delta_state = delta_state
        self.path,_ = dubins.path_sample((0,0,0), self.delta_state, 
            motion_primitive.turning_radius, motion_primitive.step_size)
        self.cost = dubins.path_length((0,0,0), delta_state, motion_primitive.turning_radius)
        self.path.append(tuple(self.delta_state.tolist()))

    def get_end_state(self, start_state):
        return start_state + rotate_state(self.delta_state,start_state[2])
            
    @staticmethod
    def get_xytheta_paths(plan):
        path = []
        for i in range(len(plan)-1,0,-1):
            seg, _ = dubins.path_sample(plan[i], plan[i-1], 
                motion_primitive.turning_radius, motion_primitive.step_size)
            path += seg
        return np.array(path)

class dubins_astar:
    def __init__(self, world_polys, Kp = 10000, look_ahead_dist = 0.1):
        self.world_polys = world_polys
        self.Kp = Kp
        self.look_ahead_dist = look_ahead_dist
        self.last_idx = 0

    def valid_edge(self, state, primitive):
        prim_states = [state + rotate_state(np.array(st),state[2]) for st in primitive.path]
        polygons = [Point(x, y).buffer(1) for (x,y,z) in prim_states]
        #path = LineString(prim_states)
        path = cascaded_union(polygons)
        for ob in self.world_polys:
            if path.intersects(ob):
                return False        
        return True
        
    def heuristic(self, state1, state2):
        state_diff = state1 - state2
        return np.sqrt(state_diff[0]**2 + state_diff[1]**2)

    def control_policy(self, state, path_states):
            
        curr_state = np.array([state['x'],state['y'],state['theta']])
        err_vec = path_states[self.last_idx:] - curr_state
        dists = np.sum(np.abs(err_vec)**2,axis=-1)
        idx = np.argmin(dists)
        
        self.last_idx += idx
        
        idx += 10
        #min_dist = dists[idx]
            
        #if min_dist < self.look_ahead_dist:
        #    idx += 10
        
        if idx > len(dists):
            idx = len(dists)
        

        
        err = wrapToPi(curr_state[2] - np.arctan2(err_vec[idx][1],err_vec[idx][0]))            
        action = -self.Kp*err
        action = np.median([-1, 1, action])
        return action
    
    
    def cost_function(self, state, motion_primitive):
        return motion_primitive.cost # + belief cost

    def state_equality(self, state1, state2):
        anglecheck = wrapToPi(state1[2]-state2[2])
        return np.array_equal(state1[0:2],state2[0:2]) and anglecheck<0.0001
