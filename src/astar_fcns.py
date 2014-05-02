import numpy as np
import matplotlib.pyplot as plt
import pdb
import dubins
import copy
from shapely.geometry import LineString
from shapely.ops import cascaded_union
from shapely.geometry import Point, CAP_STYLE, JOIN_STYLE, box
from shapely import affinity
from descartes import PolygonPatch
import cPickle as pickle

def wrapToPi(angle):
    return (angle + np.pi) % (2.0 * np.pi ) - np.pi

def rotate_state(state, angle):
    rotMatrix = np.array([[np.cos(angle), -np.sin(angle)],
                        [np.sin(angle),  np.cos(angle)]])
    rot_state = copy.deepcopy(state)
    rot_state[0:2] = rotMatrix.dot(state[0:2])    
    #rot_state[2] = 0.0
    return rot_state

class motion_primitive:
    turning_radius = 3.099999999 #2.999999
    step_size = 0.1
    theta_res = np.pi/4.0
    def __init__(self, delta_state, start_angle = 0, isbackward=False):
        length = 3.0
        width = 2.0
        self.delta_state = delta_state
        self.start_angle = start_angle
        self.cost = dubins.path_length((0,0,self.start_angle), delta_state, motion_primitive.turning_radius)
        path_list, _ = dubins.path_sample((0,0,self.start_angle), self.delta_state, 
            motion_primitive.turning_radius, 0.1)
        self.path = np.array(path_list)

        self.isbackward = isbackward
        if self.isbackward:
            self.delta_state[:2] = -self.delta_state[:2] #(-0.25*np.cos(start_angle),-0.25*np.sin(start_angle),start_angle)
            self.path[:,0:2] = -1*self.path[:,0:2]
            #self.path = [(-xx,-yy,tth) for (xx,yy,tth) in self.path]
            #self.cost *= 2

        box_angle_tuples = [(box(x - length/2, y - width/2, x + length/2, y + width/2), theta) for (x,y,theta) in self.path]
        polygons = [affinity.rotate(a_box, theta, origin = 'centroid', use_radians = True) for (a_box, theta) in box_angle_tuples]
        
        if False:
            fig = plt.figure(10)
            fig.clear()
            plt.ion()
            ax = fig.add_subplot(111, aspect = 'equal')
            for poly in polygons:
                P = PolygonPatch(poly, fc = 'k', zorder = 2)
                ax.add_patch(P)
            ax.set_xlim(-5,5)
            ax.set_ylim(-5,5)
            fig.show()
            

        polygons = [poly.buffer(0.1) for poly in polygons]
        try:
            self.bounding_poly = cascaded_union(polygons).simplify(0.05)
        except:
            raise Exception('No bounding poly for primitive')


    def get_end_state(self, start_state):
        #offset = np.array((start_state[0], start_state[1], 0.0))

        rotated_state = rotate_state(self.delta_state,start_state[2])
        end_state = start_state + rotated_state
        end_state[2] = wrapToPi(end_state[2])
        #pdb.set_trace()
        return end_state
    '''        
    @staticmethod
    def get_xytheta_paths(plan):
        path = []
        for i in range(len(plan)-1,0,-1):
            seg, _ = dubins.path_sample(plan[i], plan[i-1], 
                motion_primitive.turning_radius, motion_primitive.step_size)
            path += seg
        return np.array(path)
    '''
class dubins_astar:
    def __init__(self, world_points, value_fcn, Kp = 1000.0, Kd = 200.0, look_ahead_dist = 0.9):
        self.world_points = world_points
        self.Kp = Kp
        self.Kd = Kd
        self.look_ahead_dist = look_ahead_dist
        self.last_idx = 0
        self.last_err = 0.0
        self.value_fcn = value_fcn

        world_polys = [pt.buffer(0.5, 16, CAP_STYLE.square, JOIN_STYLE.bevel) for pt in world_points]
        self.world_polys = cascaded_union(world_polys).buffer(-0.49, 16, CAP_STYLE.flat, JOIN_STYLE.mitre)

    def valid_edge(self, state, primitive, plot = True):

        bounding_poly = affinity.rotate(primitive.bounding_poly, state[2], origin = (0.0, 0.0), use_radians = True)
        bounding_poly = affinity.translate(bounding_poly, state[0], state[1])

        #Drawing Primitive-TAKE OUT TODO
        if plot:
            if bounding_poly.intersects(self.world_polys):
                color = 'r'
            else:
                color = 'b'

            fig = plt.figure(2)
            fig.clear()
            plt.ion()
            ax = fig.add_subplot(111, aspect = 'equal')
            for poly in self.world_polys:
                P = PolygonPatch(poly, fc = 'k', zorder = 2)
                ax.add_patch(P)
            P = PolygonPatch(bounding_poly, fc = color, ec = '#999999', alpha = 1, zorder = 1)
            polyPatch = ax.add_patch(P)
            ax.set_xlim(0,50)
            ax.set_ylim(0,50)
            fig.show()
            plt.pause(0.1)
            #pdb.set_trace()
        
        if bounding_poly.intersects(self.world_polys):

            return False
        return True
        

    def heuristic(self, state1, state2):
        #state_diff = state1 - state2
        #return 15*np.sqrt(state_diff[0]**2 + state_diff[1]**2)
        return 1.5*self.value_fcn[np.around(state1[:2]).astype(int).tostring()]

    def control_policy(self, state, path_states):
        curr_state = np.array([state['x'],state['y'],state['theta']])
        err_vec = [[x,y] - curr_state[0:2] for [x,y,z] in path_states]
        dists = np.sqrt(np.sum(np.abs(err_vec)**2,axis=-1))
        #idx = np.argmin(dists)
        
        if np.fabs(self.last_err) < (np.pi-np.pi/2):
            self.look_ahead_dist = 0.5
        else:
            self.look_ahead_dist = 0.05
            
        while dists[self.last_idx] < self.look_ahead_dist:
            self.last_idx += 1
        
        if self.last_idx > len(dists):
            self.last_idx = len(dists)
        
        #self.last_idx += idx
        
        err = wrapToPi(curr_state[2] - np.arctan2(err_vec[self.last_idx][1],err_vec[self.last_idx][0]))
        if np.fabs(err) < (np.pi-np.pi/2):
            action = -self.Kp*err - self.Kd*(err-self.last_err)  + path_states[self.last_idx][2]
            action = np.median([-1, 1, action])
        else:
            action = -2
            
        self.last_err = err
        return action
    
    
    def cost_function(self, state, motion_primitive):
        return motion_primitive.cost 

    def state_equality(self, path, state2):
        if path is not None:
            for state in path:
                if np.linalg.norm(state[0:2]- state2[0:2]) < 1.0:
                    return True
        return False
