# A* Shortest Path Algorithm
# http://en.wikipedia.org/wiki/A*
# FB - 201012256
from heapq import heappush, heappop # for priority queue
import math
import time
import random

import pdb
import scipy.io as sio
from shapely.geometry import Point
from shapely.geometry import box
from shapely.ops import cascaded_union
from matplotlib import pyplot
from descartes import PolygonPatch
from shapely.geometry import CAP_STYLE, JOIN_STYLE
from shapely.ops import unary_union
from shapely.geometry import LineString

class node:
    def __init__(self, xPos, yPos, theta, parent):
        self.xPos = xPos
        self.yPos = yPos
        self.theta = theta
        self.g = None
        self.h = None
        self.f = None
        self.parent = parent
    def __cmp__(self, other):
        return cmp(self.f, other.f)

class AStar:
    def __init__:
        PQ = []
        PQ_hash = {}
        V = {}

    def heuristic(self, cur_node, goal):
       n_point = (cur_node.xPos, cur_node.yPos)
       goal_point = (goal.xPos, goal.yPos)
       return numpy.linalg.norm(goal_point-n_point)

    def getChildren(self, cur_node):
        child1 = node(cur_node.xPos, cur_node.yPos+1.0, 0.0, cur_node)
        child2 = node(cur_node.xPos+1.0, cur_node.yPos, 0.0, cur_node)
        child3 = node(cur_node.xPos, cur_node.yPos-1.0, 0.0, cur_node)
        child4 = node(cur_node.xPos-1.0, cur_node.yPos, 0.0, cur_node)
        children = [child1, child2, child3, child4]
        return children

    def cost(self, parent, child):
        return 1.0


    def plan(self, start, goal):
        n0 = node(start[0],start[1], 0.0, 0.0, h0, None)
        h0 = heuristic(n0, goal)
        PQ.heappush(n0)
        PQ_hash.update((n0.xPos, n0.yPos, n.theta),n0)

        while(PQ.shape[0] > 0):
            current = PQ.heappop()
            del PQ_hash((current.xPos, current.yPos, current.theta))
            if(atGoal(current)):
                return reconstruct_path(V, goal)

            V.update{(current.xPos,current.yPos,current.theta), current}

            #get children

            children = getChildren(current)#do set parent, should return an array of nodes
            
            for child in children:
                child_key = (child.xPos, child.yPos, child.theta)
                if child_key in V:
                    continue
                
                tentative_g = current.g + path(current,child)#add belief later

                if (child_key in PQ_hash):
                    existing_child = (PQ_hash.get(child_key))
                    g_val = existing_child.g
                    if(tentative_g >= g_val):
                        continue
                    else:
                        existing_child.g = tentative_g
                        existing_child.f = existing_child.g + heuristic(existing_child, goal)
                        existing_child.parent = current
                else:
                    child.g = tentative_g
                    child.f = child.g + heuristic(child, goal)
                    child.parent = current
        
        return None   

                        


'''
class WorldMap:
    def __init__(self, map_name):
        map_name_full = '../matlab_files/'+map_name+'.mat'
        world_map = sio.loadmat(map_name_full, squeeze_me = True)['map_struct'].item()
        world_map_1 = world_map[3]
        self.start = world_map[5].item()
        self.goal = world_map[6].item()
        print self.start
        print self.goal
        walls_list = []
        obstacles_list = []
        for x in range(0, world_map_1.shape[0]):
            for y in range(0, world_map_1.shape[1]):
                if world_map_1[y,x] == 0.0:
                    walls_list.append((y,x))
                if (world_map_1[y,x] > 0.0) and (world_map_1[y,x] < 1.0):
                    obstacles_list.append([y,x,world_map_1[y,x]])

        walls_polygons = [Point(x,y).buffer(0.5,16,CAP_STYLE.square,JOIN_STYLE.bevel) for (y,x) in walls_list]
        self.walls_map = cascaded_union(walls_polygons)#list of polygons

        obstacles_polygons = [Point(x,y).buffer(0.5,16,CAP_STYLE.square,JOIN_STYLE.bevel) for (y,x,z) in obstacles_list]
        self.obstacles_map = obstacles_polygons
        
        self.fig = pyplot.figure(1,figsize=[14,7],dpi = 90)
        self.ax = self.fig.add_subplot(111)
        xrange = [-0.5,49.5]
        yrange = [-0.5,49.5]
        self.ax.set_xlim(*xrange)
        self.ax.set_ylim(*yrange)
        self.ax.set_aspect(1)
        for walls in self.walls_map:
            p = PolygonPatch(walls, fc='#000000', ec='#000000', alpha=1.0, zorder=1)
            self.ax.add_patch(p)
        counter = 0
        for ob in self.obstacles_map:
            gamma = obstacles_list[counter][2]
            p = PolygonPatch(ob, fc='#000000', ec='#000000', alpha=1-gamma, zorder=1)
            self.ax.add_patch(p)
            counter = counter + 1
        start_polygons = Point(self.start[0],self.start[1]).buffer(0.25)
        goal_polygons = Point(self.goal[0],self.goal[1]).buffer(0.25)
        p = PolygonPatch(start_polygons, fc='#ff0000', ec='#ff0000', alpha=1, zorder=1)
        self.ax.add_patch(p)
        p = PolygonPatch(goal_polygons, fc='#00ff00', ec='#00ff00', alpha=1, zorder=1)
        self.ax.add_patch(p)
        
        carWidth = 2.0
        carHeight = 3.0
        self.car_poly = box(-carHeight/2.0,-carWidth/2.0,carHeight/2.0,carWidth/2.0)
        

    def visualize(self, carX, carY, carTheta):
        p = PolygonPatch(self.car_poly, fc='#0000ff', ec='#0000ff', alpha = 1.0, zorder = 2)
        self.ax.add_patch(p)
        pyplot.show() 


if __name__ == "__main__":
    a = WorldMap('map_1')
    a.visualize(2.0, 2.0, 0.0)
class AStar:
    #world_map is a cascaded union of
    def __init__(world_map, node0):
        PQ = []#MAY BE COMPLETE BS
        V = []


    def heuristic(state, goal):
class node:
    xPos = 0.0 # x position
    yPos = 0.0 # y position
    theta = 0.0
    #distance = 0 # total distance already travelled to reach the node
    #priority = 0 # priority = distance + remaining distance estimate
    def __init__(self, xPos, yPos, distance, priority):
        self.xPos = xPos
        self.yPos = yPos
        #self.distance = distance
        #self.priority = priority
    #def __lt__(self, other): # comparison method for priority queue
        #return self.priority < other.priority
    def updatePriority(self, xDest, yDest):
        self.priority = self.distance + self.estimate(xDest, yDest) * 10 # A*
    # give higher priority to going straight instead of diagonally
    def nextMove(self, dirs, d): # d: direction to move
        if dirs == 8 and d % 2 != 0:
            self.distance += 14
        else:
            self.distance += 10
    # Estimation function for the remaining distance to the goal.
    def estimate(self, xDest, yDest):
        xd = xDest - self.xPos
        yd = yDest - self.yPos
        # Euclidian Distance
        d = math.sqrt(xd * xd + yd * yd)
        # Manhattan distance
        # d = abs(xd) + abs(yd)
        # Chebyshev distance
        # d = max(abs(xd), abs(yd))
        return(d)

# A-star algorithm.
# The path returned will be a string of digits of directions.
def pathFind(the_map, n, m, dirs, dx, dy, xA, yA, xB, yB):
    closed_nodes_map = [] # map of closed (tried-out) nodes
    open_nodes_map = [] # map of open (not-yet-tried) nodes
    dir_map = [] # map of dirs
    row = [0] * n
    for i in range(m): # create 2d arrays
        closed_nodes_map.append(list(row))
        open_nodes_map.append(list(row))
        dir_map.append(list(row))

    pq = [[], []] # priority queues of open (not-yet-tried) nodes
    pqi = 0 # priority queue index
    # create the start node and push into list of open nodes
    n0 = node(xA, yA, 0, 0)
    n0.updatePriority(xB, yB)
    heappush(pq[pqi], n0)
    open_nodes_map[yA][xA] = n0.priority # mark it on the open nodes map

    # A* search
    while len(pq[pqi]) > 0:
        # get the current node w/ the highest priority
        # from the list of open nodes
        n1 = pq[pqi][0] # top node
        n0 = node(n1.xPos, n1.yPos, n1.distance, n1.priority)
        x = n0.xPos
        y = n0.yPos
        heappop(pq[pqi]) # remove the node from the open list
        open_nodes_map[y][x] = 0
        closed_nodes_map[y][x] = 1 # mark it on the closed nodes map

        # quit searching when the goal is reached
        # if n0.estimate(xB, yB) == 0:
        if x == xB and y == yB:
            # generate the path from finish to start
            # by following the dirs
            path = ''
            while not (x == xA and y == yA):
                j = dir_map[y][x]
                c = str((j + dirs / 2) % dirs)
                path = c + path
                x += dx[j]
                y += dy[j]
            return path

        # generate moves (child nodes) in all possible dirs
        for i in range(dirs):
            xdx = x + dx[i]
            ydy = y + dy[i]
            if not (xdx < 0 or xdx > n-1 or ydy < 0 or ydy > m - 1
                    or the_map[ydy][xdx] == 1 or closed_nodes_map[ydy][xdx] == 1):
                # generate a child node
                m0 = node(xdx, ydy, n0.distance, n0.priority)
                m0.nextMove(dirs, i)
                m0.updatePriority(xB, yB)
                # if it is not in the open list then add into that
                if open_nodes_map[ydy][xdx] == 0:
                    open_nodes_map[ydy][xdx] = m0.priority
                    heappush(pq[pqi], m0)
                    # mark its parent node direction
                    dir_map[ydy][xdx] = (i + dirs / 2) % dirs
                elif open_nodes_map[ydy][xdx] > m0.priority:
                    # update the priority
                    open_nodes_map[ydy][xdx] = m0.priority
                    # update the parent direction
                    dir_map[ydy][xdx] = (i + dirs / 2) % dirs
                    # replace the node
                    # by emptying one pq to the other one
                    # except the node to be replaced will be ignored
                    # and the new node will be pushed in instead
                    while not (pq[pqi][0].xPos == xdx and pq[pqi][0].yPos == ydy):
                        heappush(pq[1 - pqi], pq[pqi][0])
                        heappop(pq[pqi])
                    heappop(pq[pqi]) # remove the target node
                    # empty the larger size priority queue to the smaller one
                    if len(pq[pqi]) > len(pq[1 - pqi]):
                        pqi = 1 - pqi
                    while len(pq[pqi]) > 0:
                        heappush(pq[1-pqi], pq[pqi][0])
                        heappop(pq[pqi])       
                    pqi = 1 - pqi
                    heappush(pq[pqi], m0) # add the better node instead
    return '' # if no route found

# MAIN
dirs = 8 # number of possible directions to move on the map
if dirs == 4:
    dx = [1, 0, -1, 0]
    dy = [0, 1, 0, -1]
elif dirs == 8:
    dx = [1, 1, 0, -1, -1, -1, 0, 1]
    dy = [0, 1, 1, 1, 0, -1, -1, -1]

n = 30 # horizontal size of the map
m = 30 # vertical size of the map
the_map = []
row = [0] * n
for i in range(m): # create empty map
    the_map.append(list(row))

# fillout the map with a '+' pattern
for x in range(n / 8, n * 7 / 8):
    the_map[m / 2][x] = 1
for y in range(m/8, m * 7 / 8):
    the_map[y][n / 2] = 1

# randomly select start and finish locations from a list
sf = []
sf.append((0, 0, n - 1, m - 1))
sf.append((0, m - 1, n - 1, 0))
sf.append((n / 2 - 1, m / 2 - 1, n / 2 + 1, m / 2 + 1))
sf.append((n / 2 - 1, m / 2 + 1, n / 2 + 1, m / 2 - 1))
sf.append((n / 2 - 1, 0, n / 2 + 1, m - 1))
sf.append((n / 2 + 1, m - 1, n / 2 - 1, 0))
sf.append((0, m / 2 - 1, n - 1, m / 2 + 1))
sf.append((n - 1, m / 2 + 1, 0, m / 2 - 1))
(xA, yA, xB, yB) = random.choice(sf)

print 'Map size (X,Y): ', n, m
print 'Start: ', xA, yA
print 'Finish: ', xB, yB
t = time.time()
route = pathFind(the_map, n, m, dirs, dx, dy, xA, yA, xB, yB)
print 'Time to generate the route (seconds): ', time.time() - t
print 'Route:'
print route

# mark the route on the map
if len(route) > 0:
    x = xA
    y = yA
    the_map[y][x] = 2
    for i in range(len(route)):
        j = int(route[i])
        x += dx[j]
        y += dy[j]
        the_map[y][x] = 3
    the_map[y][x] = 4

# display the map with the route added
print 'Map:'
for y in range(m):
    for x in range(n):
        xy = the_map[y][x]
        if xy == 0:
            print '.', # space
        elif xy == 1:
            print 'O', # obstacle
        elif xy == 2:
            print 'S', # start
        elif xy == 3:
            print 'R', # route
        elif xy == 4:
            print 'F', # finish
    print

raw_input('Press Enter...')
'''
