import pygame
import sys,os
import numpy as np
import packages.RobotPlanningRoutines.ObstaclesFactory as factory
from   packages.RobotPlanningRoutines.planners_and_env import EnvMap,RRTplanner,Robot
from   packages.RobotPlanningRoutines.CollisionChecks import CircleCollision,GJK
import time

pygame.init()
#pygame.font.init()
#myfont = pygame.font.SysFont('Comic Sans MS', 30)

## Simulates a set of simulations in a loop

output_dir = "./OutputSimulationsMap7_RebaseRad"
map_dir    = "./packages/maps"


map_name           = os.path.join(map_dir,'MAP7.txt')


dimensions = (1200,600)    # map dimension
start      = (100, 50,0)  # start configuration
goal       = (1100, 500,0) # goal configuration

maxRRTstep = 50           # define step in the RRT search
rebase_rad = range(100,800,50)

output_file_names  = [os.path.join(output_dir,'Simulation'+str(num)+'_RebaseRad_'+str(step)+'.txt') for num,step in enumerate(rebase_rad)]

i             = 0             # counter
robot_rad     = 20
is_path_found = False
path_number   = 0


# instance map and robot 
motionMap            = EnvMap(start,goal,dimensions)
myrobot              = Robot(start[0],start[1],start[2],dimension=(45,25))


# DEFINE CAR MODEL
myrobot.set_car_spec(vel_max=40,max_yaw_rate=60*np.pi/180)                  # m/s
myrobot.set_baseline(baseline=45) # same as cal width

for step,outputfile in zip(rebase_rad,output_file_names ) :
    
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
    RRTpathFinder.activate_rebasing(step) # actvates rebasing   good between 100-500

    goalArea = pygame.Rect((goal[0]-maxRRTstep/2,goal[1]-maxRRTstep/2),(maxRRTstep,maxRRTstep))
    pygame.draw.rect(motionMap.map,(0,0,200,0.9),goalArea)

    # save best paths as output

    
    max_iterations  = 3000
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
                current_best_path,current_best_actions,total_cost = RRTpathFinder.getFinalPath()
                motionMap.drawPath(current_best_path)
                RRTpathFinder.save_optimal_path(current_best_path,total_cost,current_best_actions,t_end-RRTpathFinder.t_start)
                number_of_paths += 1
            i += 1
        
        pygame.display.update()
    motionMap.map.fill(motionMap.white)
    RRTpathFinder.stop_clock()
    RRTpathFinder.write_summary2txt(outputfile)


        




