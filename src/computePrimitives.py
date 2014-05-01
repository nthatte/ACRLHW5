import numpy as np
import dubins
from astar_fcns import motion_primitive
import pdb

def computePrimitives():
    [x,y,th] = np.mgrid[-3.5:3.5:7j, -3.5:3.5:7j, -np.pi:(0.75*np.pi):8j]

    x = x.flatten()
    y = y.flatten()
    th = th.flatten()

    turning_radius = 3.4999999999 #2.999999

    print 'Generating motion primitives...'
    motion_primitives = {}

    for start_angle in np.arange(-np.pi,np.pi,motion_primitive.theta_res):
        print '\n'
        print np.around(start_angle/motion_primitive.theta_res)
        mps = []
        for delta_state in zip(x,y,th):
            #print delta_state
            length = dubins.path_length((0,0,start_angle), delta_state, turning_radius)
            if length < np.pi*turning_radius and length > 0.005:
                print length, start_angle, delta_state
                mp = motion_primitive(np.array(delta_state),start_angle)
                if mp.bounding_poly is not None:
                    mps.append(mp)
                else:
                    print "failed check"
                    
        # Add reverse to each layer
        delta_state = (np.cos(start_angle),np.sin(start_angle),start_angle)
        mp = motion_primitive(np.array(delta_state),start_angle, True)
        print mp.cost, mp.start_angle, mp.delta_state
        print mp.path

        mps.append(mp)
        #pdb.set_trace()
        
        print len(mps)
        motion_primitives[np.around(start_angle/motion_primitive.theta_res)] = mps
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
