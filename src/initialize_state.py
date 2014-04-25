# Initialize observed map
observed_map = map_struct['seed_map']

goal = map_struct['goal'];

state = {}
state['x'] = map_struct['start'].item().item()[0];
state['y'] = map_struct['start'].item().item()[1];

state['theta'] = 0 # theta always defaults to 0

state['H'] = numpy.array([[numpy.cos(state['theta']), -numpy.sin(state['theta']), state['x']],
                          [numpy.sin(state['theta']),  numpy.cos(state['theta']), state['y']],
                          [                        1,                          1,         1]])
state['border'] = state['H']*params['border']

state['moveCount'] = 0;    

# reset flags
flags = 0;
