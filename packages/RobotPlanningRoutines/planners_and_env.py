import pygame
import sys
import numpy as np

from   scipy.spatial import distance
from   random import randint


class EnvMap():
    def __init__(self) :
        self.obstacle_list= None
        pass
    
    def add_obstacles(self,new_list_of_obstacles) :
        # ----------DESCRIPTION--------------
        # add obstacles to current obstacles list
        # -----------INPUT-------------------
        #  new_list_of_obstacles     ---> list of dictionaries
        # -----------OUTPUT------------------
        # update state 
        
        self.obstacle_list=new_list_of_obstacles
    
    def reset_obstacles(self,new_list_of_obstacles) :
        # ----------DESCRIPTION--------------
        # reset obstacles
        # -----------INPUT-------------------
        #  new_list_of_obstacles     ---> list of dictionaries
        # -----------OUTPUT------------------
        # update state 
        
        self.obstacle_list=new_list_of_obstacles
    
    def get_centers(self):
        # ----------DESCRIPTION--------------
        # obtain obsracles centers
        # -----------INPUT-------------------
        # NONE
        # -----------OUTPUT------------------
        # centers                     ---> list of tuple
        
        centers = [tuple(obstacle['centers']) for obstacle in self.obstacle_list ]
        
        return centers 
            
    def draw_obstacles(self,screen) :
        
        # ----------DESCRIPTION--------------
        # draw obstacles in the screen
        # -----------INPUT-------------------
        # screen             ----> pygame.screen object
        # -----------OUTPUT------------------

        
        for obstacle in self.obstacle_list :
            if obstacle['type'] == 'circle' :
                pygame.draw.circle(screen,obstacle['color'],tuple(obstacle['center']),obstacle['radius'])
            else :
                pygame.draw.polygon(screen,obstacle['color'],tuple(obstacle['vertices']))
            
        
        
###############################################################################################
# SIMPLE ROBOT CLASS
###############################################################################################

# create instacle of obstacles 
# to have in the robot class

# TODO :
# 1) add class inheritance as soon as you 
# understand it and put it in Robot
# 2) add other shapes to the ones that are available
#   for now only square is available

class Robot():
    # initialise the robot
    def __init__(self,x0=0,y0=0,thrust=10,type='circle',dimension=20) :
        
        # state of the system
        self.x0      = x0    # m     x of the baricenter
        self.y0      = y0    # m     y of the baricenter
        self.vx0     = 0.0   # m/s
        self.vy0     = 0.0   # m/s
        self.state   = np.array([self.x0,self.y0,self.vx0,self.vy0])
         
        # dynamic parameters 
        self.thrust  = thrust # N  
        self.dt      = 0.01 # s         # integration step
        self.mass    = 10   # kg        # mass
        self.gravity = 9.81 # pixel/s2      # gravity acceleration
        self.type    = 'type'   
        
        self.radius  = 20 # add capability to change it 
        # available_shapes = {'rectangle','circle','polygon'}
        # factory          = ObstaclesFactory()  
        # if self.type == 'rectangle' :
        #     pass
                      
    #  here the equaltions of motion are defined
    def motion(self) :
        
        # ----------DESCRIPTION--------------
        # define one step for the dynamical
        # model of the robot
        # -----------INPUT-------------------
        # NONE
        # -----------OUTPUT------------------
        # updates state of the system
        
        
        # equations of motion for the mall
        vmax = 20  # m/s # upper limit of the speed
        vmin = 0   # m/s # loer limit of the speed
        self.vx0 = self.vx0 + self.thrust/self.mass*self.dt
        self.vy0 = self.vy0 + self.gravity*self.dt
        # check speed
        if np.abs(self.vx0) > vmax :
            self.thrust  = 0
        if np.abs(self.vy0) > vmax :
            self.gravity  = 0 
            
        self.x0  = self.x0  + self.vx0*self.dt
        self.y0  = self.y0  + self.vy0*self.dt
        
        
    # draw the robot on the screen
    def draw_robot(self,screen):
        # ----------DESCRIPTION--------------
        # draw robot state on the screen
        # -----------INPUT-------------------
        # screen             ----> pygame.screen object
        # -----------OUTPUT------------------
        # updates state of the system
        
        color  = (10,230,100)
        center = (self.x0,self.y0)
        pygame.draw.circle(screen,color,center,self.radius)
    

    def draw_laser(self,screen,centers):
        # ----------DESCRIPTION--------------
        # draw a laser beam connecting the robot
        # center to the obstacles
        # -----------INPUT-------------------
        # screen                          ----> pygame.screen object
        # centers  obstacles centers      ----> list of tuple
        # -----------OUTPUT------------------
        # updates state of the system
        color=(255,0,0)
        for center in centers :
            robot_pos  = (self.x0,self.y0)
            pygame.draw.line(screen,color,(self.x0,self.y0),center)