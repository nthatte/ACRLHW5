def display_environment(x, y, state, observed_map, scale, DISPLAY_TYPE):
    plt.ion()
    fig = plt.figure(1)
    fig.clear()
    ax = fig.add_subplot(111, aspect = 'equal')
    if (DISPLAY_TYPE):
        plt.imshow(scipy.misc.imresize(observed_map, scale, interp='nearest'))
    else:
        ind = numpy.where(observed_map == 0)
        plt.plot(x[ind]*scale,y[ind]*scale,'k.',markersize=0.5*scale)

    plt.plot(scale*map_struct['start']['x'],scale*map_struct['start']['y'],'g.', markersize = 2*scale)
    plt.plot(scale*map_struct['goal']['x'] ,scale*map_struct['goal']['y'] ,'r.', markersize = 2*scale)

    plt.plot(scale*numpy.array([state['border'][0,:], state['border'][0,1:-1]]), 
        scale*numpy.array([state['border'][1,:], state['border'][1,1:-1]]), color='r')

    plt.plot(scale*state['x'],scale*state['y'],'b.', markersize = 2*scale)

    plt.plot(scale*numpy.array([state['x'], state['x'] 
            + params['length']/2*numpy.cos(state['theta'])]),
        scale*numpy.array([state['y'], state['y'] 
            + params['length']/2*numpy.sin(state['theta'])]),color = 'b')

    plt.axis((0, 500, 0, 500))
    plt.axis('off')
    plt.show()
