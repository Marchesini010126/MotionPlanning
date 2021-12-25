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
    def __init__(self,x0=0,y0 = 0,type='circle',dimension=20) :
        
        # state of the system
        self.x0      = x0    # m     x of the baricenter
        self.y0      = y0    # m     y of the baricenter
        self.vx0     = 0.0   # m/s
        self.yaw0    = 0.0   # rad   angle of the car with the global frame (0 = aligned with x-axis)
        self.angle0  = 0.0   # rad   angle of the wheels with local x-axis (0 = forward, negative = left).
                             #       Combined w/ velocity it causes yaw rate
        self.state   = np.array([self.x0,self.y0,self.vx0,self.angle0])
         
        # dynamic parameters
        self.dt        = 0.01 # s         # integration step
        self.mass      = 10   # kg        # mass
        self.type      = 'type'
        self.max_angle = np.pi / 8        # rad   max steering angle of the wheels
        self.radius    = 20               # add capability to change it
        self.L         = 3                # m   length of wheel base
        # available_shapes = {'rectangle','circle','polygon'}
        # factory          = ObstaclesFactory()  
        # if self.type == 'rectangle' :
        #     pass
                      
    #  here the equations of motion are defined
    def motion(self) :
        
        # ----------DESCRIPTION--------------
        # define one step for the dynamical
        # model of the robot
        # -----------INPUT-------------------
        # NONE
        # -----------OUTPUT------------------
        # updates state of the system

        # Inspired by https://github.com/winstxnhdw/KinematicBicycleModel/blob/main/kinematic_model.py

        x_dot = self.vx0 * np.cos(self.yaw0)
        y_dot = self.vx0 * np.sin(self.yaw0)
        yaw_rate = self.vx0 * np.tan(self.angle0) / self.L

        # Compute the final state
        self.x0 += x_dot * self.dt
        self.y0 += y_dot * self.dt
        self.yaw0 += yaw_rate * self.dt

    def change_angle(self, angle):
        max_angle = self.max_angle

        # Keep angle within allowed angles
        if angle > max_angle:
            angle = max_angle
        elif angle < -max_angle:
            angle = -max_angle

        # Update the angle
        self.angle0 = angle
        # Update the state
        self.state = np.array([self.x0,self.y0,self.vx0,self.angle0])
        
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