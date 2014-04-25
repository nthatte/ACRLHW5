#!/usr/bin/env python
import scipy.io as sio
from shapely.geometry import Point
from shapely.geometry import box
from shapely.ops import cascaded_union
from matplotlib import pyplot
from descartes import PolygonPatch
from shapely.geometry import CAP_STYLE, JOIN_STYLE
from shapely.ops import unary_union
from shapely.geometry import LineString

map_name_full = '../matlab_files/map_1.mat'
world_map = sio.loadmat(map_name_full, squeeze_me = True)
world_map = world_map['map_struct']['seed_map'].item()

tuple_list = []
for x in range(0,world_map.shape[0]):
    for y in range(0,world_map.shape[1]):
        if world_map[y][x] == 0.0:
            tuple_list.append((y,x))

d=0.2
polygons = [Point(x,y).buffer(0.5,16,CAP_STYLE.square,JOIN_STYLE.bevel) for (y,x) in tuple_list]
#polygons = [box(x-d,y-d,x+d,y+d) for (y,x) in tuple_list]

fig = pyplot.figure(1,figsize = [14,7],dpi = 90)

ax = fig.add_subplot(111)

d1 = cascaded_union(polygons)

a = LineString([(0,0),(10,10)])
p = PolygonPatch(a.buffer(0.1), fc='#999999', ec='#999999', alpha=1, zorder=2)
ax.add_patch(p)

for ob in d1:
    p = PolygonPatch(ob, fc='#999999', ec='#999999', alpha=0.5, zorder=1)
    ax.add_patch(p)
    print a.intersects(ob)



xrange = [0, 50]
yrange = [0, 50]

ax.set_xlim(*xrange)
ax.set_ylim(*yrange)
ax.set_aspect(1)

pyplot.show()
