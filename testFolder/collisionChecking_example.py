import numpy as np
import matplotlib.pyplot as plt
import os,sys
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection


sys.path.append(os.path.abspath('..'))

import packages.RobotPlanningRoutines.obstacles_factory as factory
from packages.RobotPlanningRoutines.planners_and_env import EnvMap
from packages.RobotPlanningRoutines.collision_checks  import CircleCollision
from packages.RobotPlanningRoutines.collision_checks  import GJK


# Use the simple circcle collision check between two polygonal obstacles

n_sides       = 3
center1       = np.array([1,2])
center2       = np.array([3,2])
bound_radius  = 3
color         = (10,200,0)

# obstacles definition
polygon1       = factory.createNpolygon(n_sides,bound_radius,center1,color)
polygon2       = factory.createNpolygon(n_sides,2*bound_radius,center2,color)

# obtain vertices

vertices_pol1 = polygon1['vertices']
vertices_pol2 = polygon2['vertices']


p1 = Polygon(vertices_pol1, facecolor = 'k')
p2 = Polygon(vertices_pol2, facecolor = 'k')

pp = PatchCollection([p1,p2], alpha=1)

fig,ax = plt.subplots()
circle1=plt.Circle(tuple(polygon1['center']),polygon1['radius'],fill=False)
circle2=plt.Circle(tuple(polygon2['center']),polygon2['radius'],fill=False)
ax.add_patch(circle1)
ax.add_patch(circle2)
ax.add_collection(pp)
ax.set_xlim([-10,10])
ax.set_ylim([-10,10])
plt.show()


## Check collision using bounding circles 

bound_radius1 = polygon1['radius']
bound_radius2 = polygon2['radius']

checkCollsion    = CircleCollision(bound_radius1,bound_radius2,center1,center2)
checkCollsionGJK = GJK(vertices_pol1 ,vertices_pol2)
print('Circle Collision Result  : {}'.format(checkCollsion))
print('GJK Collision Result     : {}'.format(checkCollsionGJK))


## check single point collision with a polygon

