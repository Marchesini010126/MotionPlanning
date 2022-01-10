import numpy as np

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
import sys,os


sys.path.append(os.path.abspath('../packages/RobotPlanningRoutines'))

import packages.RobotPlanningRoutines.ObstaclesFactory as factory
from packages.RobotPlanningRoutines.CollisionChecks  import CircleCollision
from packages.RobotPlanningRoutines.CollisionChecks  import GJK,minkowskiDifference



# Use the simple circcle collision check between two polygonal obstacles

n_sides       = 3
center1       = np.array([0,0])
bound_radius  = 30
color         = (10,200,0)

# obstacles definition
polygon1       = factory.createRectangle(10,60,center1,color)
point_check    = np.array([17,0])

# obtain vertices

vertices_pol1 = polygon1['vertices']



p1 = Polygon(vertices_pol1, facecolor = 'k')

mkdiff = minkowskiDifference(vertices_pol1,point_check)

p2 = Polygon(mkdiff, facecolor = 'k')

fig,ax = plt.subplots()
ax.add_patch(p1)
print(type(polygon1['vertices']))
circle1=plt.Circle(tuple(polygon1['center']),polygon1['radius'],fill=False)
circle2=plt.Circle(tuple(point_check),10,fill=True)
ax.add_patch(circle1)
ax.add_patch(circle2)
ax.set_xlim([-60,60])
ax.set_ylim([-60,60])
plt.show()




## Check collision using bounding circles 

bound_radius1 = polygon1['radius']

checkCollsionGJK = GJK(vertices_pol1 ,point_check,10)
print('GJK Collision Result     : {}'.format(checkCollsionGJK))


## check single point collision with a polygon

