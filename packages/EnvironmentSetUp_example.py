import pygame
import sys
import numpy as np
import RobotPlanningRoutines.ObstaclesFactory as factory
from RobotPlanningRoutines.planners_and_env import EnvMap
from RobotPlanningRoutines.CollisionChecks import CircleCollision


# THE FOLLOWING SCRIPTS INTRIDUCES THE READER TO THE 
# RobotMotionPlanning MAIN FUNCTIONALITIES.
# PYGAME IS USED AS A GRAPHIC INTERFACE FOR BUILDING
# THE MAPS IN THE ENVIRONMENT

#  Step1 : Initialise map and insert obstacles

# initialise pygame enviromemnt

screen_width  = 800 
screen_height = 600
screen        = pygame.display.set_mode((screen_width, screen_height)) 
clock         = pygame.time.Clock() # clock object to monitor data framing 
running       = True                # infinite loop initialization parameter

# create obstacles 


n_sides      = 3
center       = np.array([200,200])
bound_radius = 60
color        = (100,200,0)

# obstacles definition
polygon1       = factory.createNpolygon(n_sides,bound_radius,center,color)
circle1        = factory.createCircle(bound_radius,center+1.5*center,color)
rect1          = factory.createRectangle(200,100,center*2.1,(100,0,90))

# rotate an obstacle of a given angle
rect_rot       = factory.rotation(rect1,np.pi/8)
obstacles_list = [polygon1,circle1,rect_rot]


# An obstacle is a dictionary with the following keys/items pair :

# keywords ---> 'center'     center                              tuple
#               'vertices'   vertices                            np.array
#               'color'      color                               tuple (R,G,B)
#               'radius'     bounding radious for the obstacle   float
#               'type'       {circle,polygon}                    string
#                     
# note : for circular obstacles the vertices variable is equal to the center location
#        and the bounding radious is euqal to the radious itself

# initialise map enviroment
motionMap = EnvMap()
motionMap.add_obstacles(obstacles_list)


while running:
    # check if quit was pressed
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
           running = False
           pygame.quit()
           sys.exit()
    
    # reset the image  
    screen.fill((0, 0, 0))
    # update the screen
    motionMap.draw_obstacles(screen)
    
    pygame.display.flip()    
    screen.fill((0,0,0))       
    
    
#  Step2 : collison checking
#  Once the environment is created, it is essential 
#  to be able to check the collision between multiple 
#  polygons 
#  two routines are implemeneted in order to do that

#  1) collison check the bounding circles for each obstacle
#  2) use advanced GJK algorithm for collision checking


bound_radius1 = 1
bound_radius2 = 1
center1       = np.array([1,0])
center2       = np.array([-1,0])

checkCollsion = CircleCollision(bound_radius1,bound_radius2,center1,center2)
print('collision : {}'.format(checkCollsion))