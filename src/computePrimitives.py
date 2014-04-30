import numpy as np
import dubins
from astar_fcns import motion_primitive
import pdb

def computePrimitives():
    [x,y,th] = np.mgrid[-3:3:5j, -3:3:5j, -np.pi:(0.5*np.pi):4j]

    x = x.flatten()
    y = y.flatten()
    th = th.flatten()

    turning_radius = 2.999999

    print 'Generating motion primitives...'
    motion_primitives = {}


    for start_angle in np.arange(-np.pi,np.pi,np.pi/2.0):
        #print '\n'
        #print np.around(start_angle/(np.pi/2.0))
        mps = []
        for delta_state in zip(x,y,th):
            #print delta_state
            length = dubins.path_length((0,0,start_angle), delta_state, turning_radius)
            if length < np.pi*turning_radius and length > 0.001:
                #print length, start_angle, delta_state
                mps.append(motion_primitive(np.array(delta_state),start_angle))
    
        motion_primitives[np.around(start_angle/(np.pi/2.0))] = mps
    '''    
    print '\n'
    i = 0
    for mps in motion_primitives:
        print 'layer ' + str(i)
        print mps
        i += 1
        for prim in motion_primitives[mps]:
            print prim.path[0], prim.path[-1]
    pdb.set_trace()
    '''
    return motion_primitives
