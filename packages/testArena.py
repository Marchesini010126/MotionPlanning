import pygame
import sys,os
import numpy as np
import RobotPlanningRoutines.ObstaclesFactory as factory
from   RobotPlanningRoutines.planners_and_env import EnvMap,RRTplanner,Robot
from   RobotPlanningRoutines.CollisionChecks import CircleCollision,GJK
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
path_number = 0
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
myrobot.reset_state(start)
myrobot.draw_robot(motionMap.map)
myrobot.init_car()


#CREATE RANDOM OBSTACLES SET

#randomObstacles_list  = motionMap.createRandomMap(10,['polygon','rectangle'],minRadius=20,maxRadius = 40)

#CREATE MAP
randomObstacles_list  = motionMap.load_room_map("./maps/MAP7.txt")

print(myrobot.radius)
motionMap.add_obstacles(randomObstacles_list)
motionMap.draw_obstacles()


RRTpathFinder         = RRTplanner(start, goal,dimensions,randomObstacles_list,maxstep=maxRRTstep,robot=myrobot)

#activate RRTstar
RRTpathFinder.activate_rebasing(500) # actvates rebasing   good between 100-500

goalArea = pygame.Rect((goal[0]-maxRRTstep/2,goal[1]-maxRRTstep/2),(maxRRTstep,maxRRTstep))
pygame.draw.rect(motionMap.map,(0,0,200,0.9),goalArea)

# save best paths as output

log_path_file = 'paths.txt'
max_iterations = 5000
number_of_paths = 1       # simple counter used to save each new path only once
    

RRTpathFinder.start_clock()  
while running and i <max_iterations:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
           running = False
           pygame.quit()
           sys.exit()
    
    
    if  i <max_iterations:      
        motionMap.draw_startgoal()
        nodes, parent = RRTpathFinder.expand()
        currentDubinPath = RRTpathFinder.local_best_path
        
        if not len(currentDubinPath)==0 : 
            pygame.draw.lines(motionMap.map, motionMap.blue,False,tuple(currentDubinPath[:,:2]))
        
        myrobot.reset_state(nodes[-1])
        if not RRTpathFinder.isOnePathFound():
           myrobot.draw_robot(motionMap.map) #only if you want to draw the robot 
        
        print('Number of nodes : {}'.format(RRTpathFinder.numberOfNodes()))
        print('Iteration       : {}/{}'.format(i,max_iterations))
        if RRTpathFinder.isOnePathFound() and RRTpathFinder.get_number_of_paths_found()==number_of_paths:
            t_end = time.time()
            current_best_path,total_cost = RRTpathFinder.getFinalPath()
            motionMap.drawPath(current_best_path)
            RRTpathFinder.save_optimal_path(current_best_path,total_cost,t_end-RRTpathFinder.t_start)
            number_of_paths += 1
        i += 1
    
    pygame.display.update()
RRTpathFinder.stop_clock()
RRTpathFinder.write_summary2txt(log_path_file)


    




