import numpy as np
import copy
import pdb
from display_environment import *
from initialize_state import *
from motionModel import *
import matplotlib.pyplot as plt
from astar import AStar
from astar_fcns import *
from MDP import MDP
from grid_mdp_fcns_uncertain import GridWorldMDP
import dubins
from shapely.geometry import Point, CAP_STYLE, JOIN_STYLE, MultiPoint
from shapely.ops import cascaded_union
import computePrimitives

#*****************************
# Load map files and parameters
#*****************************
import scipy.io as sio

map_name = 'maps/map_3'
map_struct_packed = sio.loadmat(map_name + '.mat', squeeze_me = True)['map_struct'].item()
map_struct = {}
map_struct['map_name'] = map_struct_packed[0]
map_struct['bridge_locations'] = map_struct_packed[1]
map_struct['bridge_probabilities'] = map_struct_packed[2]
map_struct['seed_map'] = map_struct_packed[3]
map_struct['map_samples'] = map_struct_packed[4]
map_struct['start'] = map_struct_packed[5].item()
map_struct['goal'] = map_struct_packed[6].item()

#load params
from load_sim_params import *

# scale is used to blow up the environment for display purposes only. Set
# to whatever looks good on your screen
scale = 10.0

# determines the size of the map and creates a meshgrid for display
# purposes
(N,M) = map_struct['seed_map'].shape
(x,y) = np.meshgrid(np.arange(1,N+1), np.arange(1,M+1))

DISPLAY_ON = 1 # 1 - turns display on, 0 - turns display off
DISPLAY_TYPE = 'dots' # display as dots or blocks


#*****************************
# Training/Learning Phase
#*****************************
#
# Here is where you perform any training or learning that your algorithm
# will require, such as solving an MDP, determining a policy or a value
# function, etc.
#
# NOTE: At this stage, you may access any variables provided in params or 
# map_struct EXCEPT:
#
#           map_struct.map_samples 
#
# This is your test data, don't touch them! You may however create your own 
# map_samples if you feel that this will be beneficial. You may use any of
# the functions provided (such as motion model) or create your own. Also
# note that this must be kept constant for any and all maps. You cannot
# hand tweak parameters or settings in your solver for specific maps.
#
# This code block is allowed to run as long as you'd like, provided that it
# finishes in time for you to submit the assignment!
#


#*****************************
# Run Sim
#*****************************
#
# Here is where you'll actually run the simulation over the set of sampled
# maps. At each iteration, you will decide which action to take based on
# the state of the car, the observed map, and any training/learning data 
# that you may have created. There is no time limit on run time however you 
# must be able to run all map samples in time to submit the assignment.
#
#
#
# Loop through each map sample
for i in range(0,len(map_struct['map_samples'])):

    # Initialize the starting car state and observed map
    # observed_map is set to seed map, and the bridge information will be
    # updated once the car is within params.observation_radius
    state, observed_map_new, flags, goal = init_state(map_struct, params)    
    observed_map_old = None

    # display the initial state
    if DISPLAY_ON:
        disp = display_environment(x, y, state, map_struct, params, 
            observed_map_new, scale, DISPLAY_TYPE = DISPLAY_TYPE)

    # loop until maxCount has been reached or goal is found
    loopCounter = 0
    invalidPath = False
    while (state['moveCount'] < params['max_moveCount']) and flags == 0:
        #---------------------------------------
        #
        #*****************************
        # Decide Action
        #*****************************
        #
        # Here you execute your policy (which may include re-planning or
        # any technique you consider necessary):
        
        #If we have just started, or if the path has become invalid due to observations,
        #we should run the AStar algorithm on the currently observed map using motion
        #primitives generated offline. If the current path is valid, we should just
        #execute the next step in it.
        if(loopCounter == 0 or invalidPath):
            print 'Planning...'
            if loopCounter == 0:

                world_map = copy.deepcopy(map_struct['seed_map'])

                tuple_list = []
                for xx in range(1,world_map.shape[0]+1):
                    for yy in range(1,world_map.shape[1]+1):
                        if world_map[yy-1][xx-1] == 0.0:
                            #tuple_list.append((yy+0.5,xx+0.5))
                            tuple_list.append((yy,xx))

                world_points = MultiPoint([Point(xx,yy) for (yy,xx) in tuple_list])

                #Set up motion primitives
                # Define primitives relative to (0,0,0)
                #delta_states = [np.array([ 1,  0, 0.0]),
                #                np.array([ 5,  0, 0.0]),
                #                np.array([ 3,  3, np.pi/2.0]),
                #                np.array([ 3, -3, -np.pi/2.0])]

                motion_primitives = computePrimitives.computePrimitives()

                #load initial value function for this map
                with open(map_name +'value.pickle', 'rb') as handle:
                    value_fcn = pickle.load(handle)

                #set up grid world mdp
                '''
                grid_mdp = GridWorldMDP(map_struct['seed_map'], map_struct['goal'])
                '''
                grid_mdp = GridWorldMDP(map_struct['seed_map'], map_struct['goal'], 
                    map_struct['start'], map_struct['bridge_probabilities'], 
                    map_struct['bridge_locations'])
                mdp = MDP(grid_mdp.states, grid_mdp.valid_actions_function, grid_mdp.cost_function)
                value_fcn = mdp.value_iteration(value = value_fcn, plot=True, world_size = 50)

                #set up dubins astar
                dub = dubins_astar(world_points, value_fcn)
                astar = AStar(motion_primitives, dub.cost_function, dub.heuristic,
                    dub.valid_edge, dub.state_equality, plot = False)

                astar_state = np.array([state['x'],state['y'],state['theta']])
            else:
                '''
                following_dist = 0.0
                temp_idx = dub.last_idx
                while following_dist < dub.look_ahead_dist
                    temp_idx -= 1
                    path_diff = numpy.array([,])
                    following_dist += np.linalg.norm(path_diff)
                index = temp_idx
                '''
                #ind_diff = np.abs(np.array(indices) - carrot_idx)
                #index = indices[np.argmin(ind_diff)]
                astar_state = np.array([state['x'],state['y'],state['theta']]) #path_states[index,:]
                invalidPath = False

            astar_goal = np.array([goal[0],goal[1],0])
            plan, cost = astar.plan(astar_state, astar_goal)
            if plan is not None:
                print 'path cost:'
                print cost
                path_states = motion_primitive.get_xytheta_paths(plan)
                dub.last_seg = 1 # not 0 since 0 is the root and has not path segment
                dub.last_idx = 0
                print 'done'
            else:
                break

        #compute action
        action = dub.control_policy(state, plan)
        
        # Execute the action and update observed_map
        observed_map_old = copy.deepcopy(observed_map_new)
        (state, observed_map_new, flags) = motionModel(params, state, action,
            observed_map_new, map_struct['map_samples'][i], goal)
    
        if not numpy.array_equal(np.ceil(observed_map_old), np.ceil(observed_map_new)):
            print 'bridge detected! Replanning...'

            world_map = observed_map_new
            tuple_list = []
            for xx in range(1,world_map.shape[0]+1):
                for yy in range(1,world_map.shape[1]+1):
                    if world_map[yy-1][xx-1] == 0.0:
                        tuple_list.append((yy,xx))

            world_points = MultiPoint([Point(xx,yy) for (yy,xx) in tuple_list])

            #load initial value function for this map
            '''
            with open(map_name +'value_blocked.pickle', 'rb') as handle:
                value_fcn = pickle.load(handle)
            '''

            #set up grid world mdp
            start_state = np.around(np.array(state['x'],state['y']))
            grid_mdp = GridWorldMDP(observed_map_new, map_struct['goal'], 
                start_state, map_struct['bridge_probabilities'], 
                map_struct['bridge_locations'])
            '''
            grid_mdp = GridWorldMDP(observed_map_new, map_struct['goal'])
            '''
            mdp = MDP(grid_mdp.states, grid_mdp.valid_actions_function, grid_mdp.cost_function)
            value_fcn = mdp.value_iteration(value = value_fcn, plot=True, world_size = 50)

            #set up dubins astar
            #carrot_idx = dub.last_idx
            dub = dubins_astar(world_points, value_fcn)
            astar = AStar(motion_primitives, dub.cost_function, dub.heuristic,
                dub.valid_edge, dub.state_equality, plot = False)
            invalidPath = True

            
        if DISPLAY_ON:
            disp.plot(x, y, state, observed_map_new, path_states, plan[dub.last_seg].path[dub.last_idx][0:2])

        # display some output
        print state['x'], state['y'], state['theta'], state['moveCount']
        
        #if loopCounter == 0:
        #    pdb.set_trace()

        loopCounter += 1
        # pause if you'd like to pause at each step
        # pause

    del(disp)
    if flags == 1:
        print 'reached goal!'
    elif flags == 2:
        print 'collision!'
