import pygame
import sys
import numpy as np
import packages.RobotPlanningRoutines.ObstaclesFactory as factory
from   packages.RobotPlanningRoutines.planners_and_env import EnvMap,RRTplanner,Robot#,#writepath2txt
from   packages.RobotPlanningRoutines.CollisionChecks import CircleCollision,GJK
import time

pygame.init()
#pygame.font.init()
#myfont = pygame.font.SysFont('Comic Sans MS', 30)

## TEST ARENA FOR THE RRT FUNCTIONALITIES
# Specifications on the robot/environment/RRTplanner


dimensions = (1200,600)    # map dimension
start      = (100, 50,0)  # start configuration
goal       = (1100, 500,0) # goal configuration

obsNum     = 200           # number of obstacles
maxRRTstep = 70           # define step in the RRT search

i             = 0             # counter
running       = True
robot_rad     = 20
is_path_found = False
target_index  = 0

# instance map and robot 
motionMap            = EnvMap(start,goal,dimensions)
myrobot              = Robot(start[0],start[1],start[2],dimension=(45,25))

# DEFINE CAR MODEL
myrobot.set_car_spec(vel_max=40,max_yaw_rate=60*np.pi/180)                  # m/s
myrobot.set_baseline(baseline=45) # same as cal width

#randomObstacles_list  = motionMap.createRandomMap(20,['polygon','rectangle'],minRadius=20,maxRadius = 40)

#CREATE MAP
randomObstacles_list  = motionMap.load_room_map("./packages/maps/MAP1.txt")

print(myrobot.radius)
motionMap.add_obstacles(randomObstacles_list)
motionMap.draw_obstacles()


RRTpathFinder         = RRTplanner(start, goal,dimensions,randomObstacles_list,maxstep=maxRRTstep,robot=myrobot)

motionMap.draw_obstacles()
myrobot.initialise_sprite("./car_sprite.png")
myrobot.reset_state(start)
myrobot.draw_robot(motionMap.map)
RRTpathFinder.activate_rebasing(500) # actvates rebasing   good between 100-500

new_path=False

goalArea = pygame.Rect((goal[0]-maxRRTstep/2,goal[1]-maxRRTstep/2),(maxRRTstep,maxRRTstep))
pygame.draw.rect(motionMap.map,(0,0,200,0.05),goalArea)


log_path_file = 'paths.txt'
max_iterations = 5000
number_of_paths = 1       # simple counter used to save each new path only once


while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
           running = False
           pygame.quit()
           sys.exit()
           
    if not RRTpathFinder.isOnePathFound():
               #myrobot.update_sprite()
               #motionMap.map.blit(myrobot.image,myrobot.rect)
               myrobot.draw_robot(motionMap.map)
              
               
    if  i <max_iterations:      
        motionMap.draw_startgoal()
        
        nodes, parent = RRTpathFinder.expand()
       
        currentDubinPath = RRTpathFinder.local_best_path
        if not len(currentDubinPath)==0 : 
            pygame.draw.lines(motionMap.map, motionMap.blue,False,tuple(currentDubinPath[:,:2]))
        
        myrobot.reset_state(nodes[-1])

        
               
        print('Number of nodes : {}'.format(RRTpathFinder.numberOfNodes()))
        print('Iteration       : {}/{}'.format(i,max_iterations))
        if RRTpathFinder.isOnePathFound() and RRTpathFinder.get_number_of_paths_found()==number_of_paths:
            t_end = time.time()

        #myrobot.draw_robot(motionMap.map) #only if you want to draw the robot 

        
        if RRTpathFinder.isOnePathFound() and RRTpathFinder.get_number_of_paths_found()==number_of_paths:
              
                current_best_path,current_best_actions,total_cost = RRTpathFinder.getFinalPath()
                motionMap.drawPath(current_best_path)
                number_of_paths += 1
        
        i += 1
    
    
    pygame.display.update()
    


    




