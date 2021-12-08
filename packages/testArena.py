import pygame
import sys
import numpy as np
import RobotPlanningRoutines.ObstaclesFactory as factory
from   RobotPlanningRoutines.planners_and_env import EnvMap,RRTGraph,Robot
from   RobotPlanningRoutines.CollisionChecks import CircleCollision,GJK
import time

dimensions = (800, 600)
start      = (0, 0)
goal       = (600, 500)
obsDim     = 50
obsNum     = 200
i          = 0
t1         = 0
coords     = []
smooth     = []
running    = True
pygame.init()

motionMap            = EnvMap(start,goal,dimensions)
myRobot              = Robot(start_loc=start,radius=20)
randomObstacles_list = motionMap.createRandomMap(33,['polygon','rectangle'])

RRTpathFinder = RRTGraph(start, goal,dimensions,motionMap.get_obstacles(),myRobot)

motionMap.draw_obstacles()

t1 = time.time()
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
           running = False
           pygame.quit()
           sys.exit()
           
    elapsed = time.time() - t1
    t1 = time.time()
    
    motionMap.draw_startgoal()
    if i % 10 == 0:
        x, y, parent = RRTpathFinder.bias(goal)
        pygame.draw.circle(motionMap.map, motionMap.grey, (x[-1], y[-1]), motionMap.nodeRad+2, 0)
        pygame.draw.line(motionMap.map, motionMap.blue, (x[-1], y[-1]), (x[parent[-1]], y[parent[-1]]), motionMap.edgeThickness)
    else:
        x, y, parent = RRTpathFinder.expand()
        pygame.draw.circle(motionMap.map, motionMap.grey, (x[-1], y[-1]), motionMap.nodeRad+2, 0)
        pygame.draw.line(motionMap.map, motionMap.blue, (x[-1], y[-1]), (x[parent[-1]], y[parent[-1]]), motionMap.edgeThickness)
    if i % 10 == 0:
        pygame.display.update()
        if RRTpathFinder.pathToGoal() :
    
    #smooth.append(RRTpathFinder.getPathCoords())

    i += 1

    motionMap.drawPath(RRTpathFinder.getPathCoords())





