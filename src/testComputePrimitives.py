from computePrimitives import *
import matplotlib.pyplot as plt
from shapely.geometry import LineString, Point
from descartes import PolygonPatch
import pdb

motion_primitives = computePrimitives()

fig = plt.figure(2)
fig.clear()
plt.ion()
ax = fig.add_subplot(111, aspect = 'equal')
for mp in motion_primitives[0.0]:
    color = '#9999FF'
    if mp.isbackward:
        color = '#99FF99'
    a = LineString(mp.path)
    p = PolygonPatch(a.buffer(0.1), fc=color, ec=color, alpha=1, zorder=2)
    ax.add_patch(p)
    p2 = PolygonPatch(Point(mp.delta_state[0:2]).buffer(0.1), fc='000000', ec='000000', alpha=1, zorder=3)
    ax.add_patch(p2)
ax.set_xlim(-3.5,3.5)
ax.set_ylim(-3.5,3.5)
ax.grid()
fig.show()
plt.pause(0.1)
pdb.set_trace()