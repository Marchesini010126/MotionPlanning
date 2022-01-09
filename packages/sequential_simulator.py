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

## Simulates a set of simulations in a loop

output_dir = './OutputSimulations'
map_dir = "./maps"
map_names         = [ os.path.join(map_dir,'MAP'+str(num)+'.txt') for num in range(1,8)]
output_file_names  = [ os.path.join(output_dir,'Simulation'+str(num)+'.txt') for num in range(8)]

dimensions = (1200,600)    # map dimension
start      = (100, 50,0)  # start configuration
goal       = (1100, 500,0) # goal configuration

maxRRTstep = 70           # define step in the RRT search

i             = 0             # counter
robot_rad     = 20
is_path_found = False
path_number   = 0


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


for map_name,outputfile in zip(map_names,output_file_names ) :
    running       = True 
    is_path_found = False
    path_number   = 0 
    i             = 0             # counter
    
    #CREATE MAP
    randomObstacles_list  = motionMap.load_room_map(map_name)
    motionMap.add_obstacles(randomObstacles_list)
    motionMap.draw_obstacles()

    RRTpathFinder         = RRTplanner(start, goal,dimensions,randomObstacles_list,maxstep=maxRRTstep,robot=myrobot)
    RRTpathFinder.set_map_name(map_name)
    
    #activate RRTstar
    RRTpathFinder.activate_rebasing(500) # actvates rebasing   good between 100-500

    goalArea = pygame.Rect((goal[0]-maxRRTstep/2,goal[1]-maxRRTstep/2),(maxRRTstep,maxRRTstep))
    pygame.draw.rect(motionMap.map,(0,0,200,0.9),goalArea)

    # save best paths as output

    
    max_iterations = 2000
    number_of_paths = 1       # simple counter used to save each new path only once
        

    RRTpathFinder.start_clock()  
    while running and i <max_iterations:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                pygame.quit()
        
        
        if  i <max_iterations:      
            motionMap.draw_startgoal()
            nodes, parent = RRTpathFinder.expand()
            currentDubinPath = RRTpathFinder.local_best_path
            
            if not len(currentDubinPath)==0 : 
                pygame.draw.lines(motionMap.map, motionMap.blue,False,tuple(currentDubinPath[:,:2]))
            
            myrobot.reset_state(nodes[-1])
            #if not RRTpathFinder.isOnePathFound():
                #myrobot.draw_robot(motionMap.map) #only if you want to draw the robot 
            
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
    motionMap.map.fill(motionMap.white)
    RRTpathFinder.stop_clock()
    RRTpathFinder.write_summary2txt(outputfile)


        




