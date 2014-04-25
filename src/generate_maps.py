def generateMaps(mapFileName, numMaps, start_state, goal_state):
    # Inputs:
    #       mapFileName - string of the file name (including local path)
    #       numMaps - number of maps to be generated
    #
    # Outputs:
    #       generatedMap_cell - cell array containing numMaps+1 maps
    #                           the first element contains the seed map
    #

    import PIL 
    import matplotlib.plyplot as plt

    map_seed = PIL.open(mapFileName).convert('L')

    bridge_ind = numpy.where(map_seed != 0 & map_seed != 1)  # locations in the map where a bridge exists
    bridge_probs = map_seed[bridge_ind]                # probability for those bridges
    N = bridge_ind.shape[0]                             # Number of bridges

    # Holds the output for the generated bridges
    generatedMap_cell = []                

    for i in range(0,numMaps):
       temp_map = map_seed
       rand_vals = np.rand(N,1)   # generate a random value for each bridge
       bridge_vals = rand_vals < bridge_probs  # are the bridges open?
       temp_map[bridge_ind] = bridge_vals  # create the maps
       generatedMap_cell[i] = temp_map # store the maps in the cell array
       
       #plot the maps for testing
       plt.figure(i)
       plt.imshow(scipy.misc.imresize(temp_map,5, interp = 'nearest'))
       
    (N, M) = map_seed.shape
    (x, y) = numpy.meshgrid(numpy.arange(1,N+1), numpy.arange(1,M+1)) 

    map_struct['map_name'] = mapFileName
    map_struct['bridge_locations'] = np.array([x(bridge_ind).T, y(bridge_ind).T])
    map_struct['bridge_probabilities'] = bridge_probs
    map_struct['seed_map'] = map_seed
    map_struct['map_samples'] = generatedMap_cell
    map_struct['start'] = start_state
    map_struct['goal'] = goal_state
