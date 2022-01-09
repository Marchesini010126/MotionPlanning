import pygame
import sys
import numpy as np
import RobotPlanningRoutines.ObstaclesFactory as factory
from   RobotPlanningRoutines.planners_and_env import EnvMap,RRTplanner,Robot#,#writepath2txt
from   RobotPlanningRoutines.CollisionChecks import CircleCollision,GJK
import time

pygame.init()
#pygame.font.init()
#myfont = pygame.font.SysFont('Comic Sans MS', 30)

## TEST ARENA FOR THE RRT FUNCTIONALITIES
# Specifications on the robot/environment/RRTplanner


dimensions = (1200,600)    # map dimension
start      = (150, 100,0)  # start configuration
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

# DEFINE MPC HORIZON
myrobot.set_n_steps(4)
myrobot.set_time_horizon(4)

# DEFINE MPC OPTIMIZATION PENALTIES
myrobot.set_control_penalty(np.eye(2)*1E1)
myrobot.set_state_penalty(np.eye(3)*1E1)

# DEFINE CAR MODEL
myrobot.set_max_yaw_rate(60*np.pi/180) #rad/s
myrobot.set_max_steering(40*np.pi/180) #rad
myrobot.set_max_speed(40)              #m/s
myrobot.set_min_speed(0)               #m/s
myrobot.init_car()


#randomObstacles_list  = motionMap.createRandomMap(10,['polygon','rectangle'],minRadius=20,maxRadius = 40)
randomObstacles_list  = motionMap.load_room_map("MAP3.txt")
motionMap.add_obstacles(randomObstacles_list)
free_obtacles = motionMap.check_feasibility()

RRTpathFinder         = RRTplanner(start, goal,dimensions,free_obtacles,maxstep=maxRRTstep,robot=myrobot)

motionMap.draw_obstacles()

myrobot.reset_state(start)
myrobot.draw_robot(motionMap.map)
new_path=False

goalArea = pygame.Rect((goal[0]-maxRRTstep/2,goal[1]-maxRRTstep/2),(maxRRTstep,maxRRTstep))
pygame.draw.rect(motionMap.map,(0,0,200,0.05),goalArea)


while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
           running = False
           pygame.quit()
           sys.exit()
    
    if  i <10000:      
        motionMap.draw_startgoal()
        
        nodes, parent = RRTpathFinder.expand()
       
        currentDubinPath = RRTpathFinder.local_best_path
        if not len(currentDubinPath)==0 : 
            pygame.draw.lines(motionMap.map, motionMap.blue,False,tuple(currentDubinPath[:,:2]))
        
        myrobot.reset_state(nodes[-1])
        #myrobot.draw_robot(motionMap.map) #only if you want to draw the robot 

        
        if RRTpathFinder.isOnePathFound():
            current_best_path,total_cost = RRTpathFinder.getFinalPath()
            motionMap.drawPath(current_best_path)
        
        #nodes, parent = RRTpathFinder.rebase()
         
        i += 1
    
    pygame.display.update()
    


    




