import matplotlib.pyplot as plt
import numpy
from scipy.misc import imresize
import time
import pdb

def display_environment(x, y, state, map_struct, params, observed_map, scale, DISPLAY_TYPE = 'blocks'):
    if plt.fignum_exists(1):
        display_environment.l3.set_xdata(scale*numpy.append(state['border'][0,:], state['border'][0,0]))
        display_environment.l3.set_ydata(scale*numpy.append(state['border'][1,:], state['border'][1,0]))

        display_environment.l4.set_xdata(scale*state['x'])
        display_environment.l4.set_ydata(scale*state['y'])

        display_environment.l5.set_xdata(scale*numpy.array([state['x'], state['x'] 
                + params['length']/2*numpy.cos(state['theta'])]))
        display_environment.l5.set_ydata(scale*numpy.array([state['y'], state['y'] 
                + params['length']/2*numpy.sin(state['theta'])]))

    else:
        plt.ion()
        display_environment.fig = plt.figure(1)
        display_environment.fig.show()
        ax = display_environment.fig.add_subplot(111, aspect = 'equal')

        if DISPLAY_TYPE == 'blocks':
            plt.imshow(imresize(observed_map, scale, interp='nearest'))
        elif DISPLAY_TYPE == 'dots':
            ind = numpy.where(observed_map == 0)
            plt.plot(x[ind]*scale,y[ind]*scale,'k.',markersize=0.5*scale)

        display_environment.l1, = plt.plot(scale*map_struct['start'][0],
            scale*map_struct['start'][1],'g.', markersize = 2*scale)
        display_environment.l2, = plt.plot(scale*map_struct['goal'][0] ,
            scale*map_struct['goal'][1] ,'r.', markersize = 2*scale)

        display_environment.l3, = plt.plot(scale*numpy.append(state['border'][0,:], state['border'][0,0]),
            scale*numpy.append(state['border'][1,:], state['border'][1,0]), color='r')

        display_environment.l4, = plt.plot(scale*state['x'], 
            scale*state['y'],'b.', markersize = 2*scale)

        display_environment.l5, = plt.plot(scale*numpy.array([state['x'], state['x'] 
                + params['length']/2*numpy.cos(state['theta'])]),
            scale*numpy.array([state['y'], state['y'] 
                + params['length']/2*numpy.sin(state['theta'])]),color = 'b')

    plt.axis((0, 500, 0, 500))
    plt.axis('off')
    plt.draw()
    plt.pause(0.0001)
