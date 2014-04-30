import matplotlib.pyplot as plt
import numpy
from scipy.misc import imresize
import time
import pdb

class display_environment:
    def __init__(self, x, y, state, map_struct, params, observed_map, scale, 
        path=numpy.zeros((1,2)), carrot_idx=0, DISPLAY_TYPE = 'blocks'):

        #set up figure
        plt.ion()
        self.fig = plt.figure(1)
        self.fig.clear
        self.fig.show()
        ax = self.fig.add_subplot(111, aspect = 'equal')

        self.DISPLAY_TYPE = DISPLAY_TYPE
        self.scale = scale

        self.start_state = map_struct['start']
        self.goal_state  = map_struct['goal']
        self.length = params['length']

        if self.DISPLAY_TYPE == 'blocks':
            self.environ = plt.imshow(imresize(observed_map, self.scale, interp='nearest'))

        elif self.DISPLAY_TYPE == 'dots':
            ind = numpy.where(observed_map == 0)
            self.environ, = plt.plot(x[ind]*self.scale,y[ind]*self.scale,'k.',
                markersize=0.5*self.scale)

        self.l1, = plt.plot(self.scale*self.start_state[0],
            self.scale*self.start_state[1],'g.', markersize = 2*self.scale)
        self.l2, = plt.plot(self.scale*self.goal_state[0] ,
            self.scale*self.goal_state[1] ,'r.', markersize = 2*self.scale)

        self.l3, = plt.plot(self.scale*numpy.append(state['border'][0,:], 
            state['border'][0,0]),
            self.scale*numpy.append(state['border'][1,:], 
            state['border'][1,0]), color='r')

        self.l4, = plt.plot(self.scale*state['x'], 
            self.scale*state['y'],'b.', markersize = 2*self.scale)

        self.l5, = plt.plot(self.scale*numpy.array([state['x'], state['x'] 
            + self.length/2*numpy.cos(state['theta'])]),
            self.scale*numpy.array([state['y'], state['y'] 
            + self.length/2*numpy.sin(state['theta'])]),color = 'b')

        self.l6, = plt.plot(self.scale*path[:,0],self.scale*path[:,1],'k.',markersize=0.5)

        self.l7, = plt.plot(self.scale*path[carrot_idx,0],
            self.scale*path[carrot_idx,1],
            '.',color='#EB8921',markersize=2*self.scale)
            
        plt.axis((0, 500, 0, 500))
        plt.axis('off')
        plt.draw()
        plt.pause(0.0001)

    def plot(self, x, y, state, observed_map, path=numpy.zeros((1,2)), carrot_idx=0):
            if self.DISPLAY_TYPE == 'blocks':
                self.environ.set_data(imresize(observed_map, self.scale, interp='nearest'))
        
            elif self.DISPLAY_TYPE == 'dots':
                ind = numpy.where(observed_map == 0)
                self.environ.set_xdata(x[ind]*self.scale)
                self.environ.set_ydata(y[ind]*self.scale)
               
            self.l3.set_xdata(self.scale*numpy.append(state['border'][0,:], state['border'][0,0]))
            self.l3.set_ydata(self.scale*numpy.append(state['border'][1,:], state['border'][1,0]))

            self.l4.set_xdata(self.scale*state['x'])
            self.l4.set_ydata(self.scale*state['y'])

            self.l5.set_xdata(self.scale*numpy.array([state['x'], state['x'] 
                    + self.length/2*numpy.cos(state['theta'])]))
            self.l5.set_ydata(self.scale*numpy.array([state['y'], state['y'] 
                    + self.length/2*numpy.sin(state['theta'])]))
            self.l6.set_xdata(self.scale*path[:,0])
            self.l6.set_ydata(self.scale*path[:,1])
            self.l7.set_xdata(self.scale*path[carrot_idx,0])
            self.l7.set_ydata(self.scale*path[carrot_idx,1])

            plt.axis((0, 500, 0, 500))
            plt.axis('off')
            plt.draw()
            plt.pause(0.0001)

    def __del__(self):
        plt.close(self.fig)

