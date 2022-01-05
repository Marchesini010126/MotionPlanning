import pygame
import sys
import numpy as np
import RobotPlanningRoutines.ObstaclesFactory as factory
from   RobotPlanningRoutines.planners_and_env import EnvMap,RRTplanner,Robot
from   RobotPlanningRoutines.CollisionChecks import CircleCollision,GJK
import time

## TEST ARENA FOR THE RRT FUNCTIONALITIES

dimensions = (1200,600)
start      = (100, 100,0)  # start configuration
goal       = (1100, 500,0) # goal configuration
obsNum     = 200
i          = 0
t1         = 0
coords     = []
smooth     = []
running    = True
robot_rad = 20
pygame.init()

motionMap            = EnvMap(start,goal,dimensions)
myRobot              = Robot(start[0],start[1],start[2])


# randomObstacles_list  = motionMap.createRandomMap(10,['polygon','rectangle'],minRadius=20,maxRadius = 40)
randomObstacles_list = motionMap.load_room_map("MAP2.txt")
motionMap.add_obstacles(randomObstacles_list)
free_obtacles = motionMap.check_feasibility()

RRTpathFinder         = RRTplanner(start, goal,dimensions,free_obtacles,myRobot)

motionMap.draw_obstacles()

print(t1)
new_path=False
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
           running = False
           pygame.quit()
           sys.exit()
           
    elapsed = time.time() - t1
    motionMap.draw_startgoal()
    nodes, parent = RRTpathFinder.expand()
    currentDubinPath = RRTpathFinder.local_best_path
    if not len(currentDubinPath)==0 : 
        pygame.draw.lines(motionMap.map, motionMap.blue,False,tuple(currentDubinPath[:,:2]))
        
    myRobot.reset_state(nodes[-1])
    myRobot.draw_robot(motionMap.map)
    
   
    
#     if i % 10 == 0:
#         if RRTpathFinder.pathToGoal() :
#              #motionMap.drawPath(RRTpathFinder.getPathCoords())
#             pass
    
    pygame.display.update()
    
#     #smooth.append(RRTpathFinder.getPathCoords())
    i += 1





