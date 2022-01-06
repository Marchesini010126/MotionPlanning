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
maxRRTstep = 150 # step in the RRT search
i          = 0
t1         = 0
coords     = []
smooth     = []
running    = True
robot_rad  = 20
is_path_found = False


pygame.init()

motionMap            = EnvMap(start,goal,dimensions)
myRobot              = Robot(start[0],start[1],start[2])


#randomObstacles_list  = motionMap.createRandomMap(10,['polygon','rectangle'],minRadius=20,maxRadius = 40)
randomObstacles_list = motionMap.load_room_map("MAP1.txt")
motionMap.add_obstacles(randomObstacles_list)
free_obtacles = motionMap.check_feasibility()

RRTpathFinder         = RRTplanner(start, goal,dimensions,free_obtacles,maxstep=maxRRTstep,robot=myRobot)

motionMap.draw_obstacles()


new_path=False
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
           running = False
           pygame.quit()
           sys.exit()
    
    if i <30000:      
        elapsed = time.time() - t1
        motionMap.draw_startgoal()
        nodes, parent = RRTpathFinder.expand()
        currentDubinPath = RRTpathFinder.local_best_path
        if not len(currentDubinPath)==0 : 
            pygame.draw.lines(motionMap.map, motionMap.blue,False,tuple(currentDubinPath[:,:2]))
        
        goalArea = pygame.Rect((goal[0]-maxRRTstep/2,goal[1]-maxRRTstep/2),(maxRRTstep,maxRRTstep))
        pygame.draw.rect(motionMap.map,(100,100,100,0.3),goalArea)
        myRobot.reset_state(nodes[-1])
        myRobot.draw_robot(motionMap.map)
        
      
        if RRTpathFinder.isOnePathFound():
            print('goalReached')
            
            current_best_path,total_cost = RRTpathFinder.getFinalPath()
            print('Current cost : {}'.format(total_cost))
            motionMap.drawPath(current_best_path)
        pygame.display.update()

        #     #smooth.append(RRTpathFinder.getPathCoords())
    
    # else :
    #     file = 'checkpath.txt'
    #     f = open(file,'w+')
       
    #     for path,node in zip(RRTpathFinder.global_path,RRTpathFinder.nodes) :
            
    #         f.write(str(node)+'\n')
    #         for cord in path :
    #            f.write(str(cord)+'\n')
    #     f.close()
            
    
    i += 1





# for jj,node in enumerate(RRTpathFinder.nodes)):
#     printer1 = 'the final node is : '+' '.join([str(a) for a in node])
#     printer2 = 'the final node is : '+' '.join([str(a) for a in node])
    




