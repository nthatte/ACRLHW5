import numpy
import copy

# Initialize observed map
def init_state(map_struct, params):
    copy_of_map_struct = copy.deepcopy(map_struct)
    observed_map = copy_of_map_struct['seed_map']

    goal = copy_of_map_struct['goal'];

    state = {}
    state['x'] = copy_of_map_struct['start'][0];
    state['y'] = copy_of_map_struct['start'][1];

    state['theta'] = 0 # theta always defaults to 0

    state['H'] = numpy.array([[numpy.cos(state['theta']), -numpy.sin(state['theta']), state['x']],
                              [numpy.sin(state['theta']),  numpy.cos(state['theta']), state['y']],
                              [                        1,                          1,         1]])
    state['border'] = numpy.dot(state['H'],params['border'])

    state['moveCount'] = 0;    

    # reset flags
    flags = 0;

    return (state, observed_map, flags, goal)
