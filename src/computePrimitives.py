import numpy as np
import dubins
from astar_fcns import motion_primitive
import pdb

def computePrimitives():
    [x,y,th] = np.mgrid[-3:3:12j, -3:3:12j, 0:(1.75*np.pi):8j]

    x = x.flatten()
    y = y.flatten()
    th = th.flatten()

    turning_radius = 2.999999

    print 'Generating motion primitives...'
    motion_primitives = {}


    for start_angle in np.arange(0,1.75*np.pi,np.pi/4.0):
        print '\n'
        mps = []
        for delta_state in zip(x,y,th):
            length = dubins.path_length((0,0,start_angle), delta_state, turning_radius)
            if length < np.pi*turning_radius:
                #print length, start_angle, delta_state
                mps.append(motion_primitive(np.array(delta_state),start_angle))
    
        motion_primitives[start_angle] = mps
    return motion_primitives
