from astar_orig import AStar

astar = AStar()
plan = astar.plan((0,0),(10,10))

for n in plan:
    print (n.xPos, n.yPos)