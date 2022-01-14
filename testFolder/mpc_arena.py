import numpy as np
import cvxpy as cvx
from   casadi import *
import matplotlib.pyplot as plt
import pygame
from   packages.mpc_test.mpc_helper_functions import Controller_mpc
import random
import packages.RobotPlanningRoutines.ObstaclesFactory as factory

# create basic environment 
screen_width  = 800 
screen_height = 600
n_obstacles   = 10
obs_radius    = 50 # you can even make a list if you 
                   # want to have a specific radious for each
# screen        = pygame.display.set_mode((screen_width, screen_height)) 
# clock         = pygame.time.Clock() # clock object to monitor data framing 
# running       = True # infinite loop initialization


# initialise mpc controller
mpc_controller = Controller_mpc()


initial_state   = np.array([2,-2,0])
target_state    = np.array([[30,30,0*np.pi/180]])
obstacles       = []

obstacles.append(factory.createCircle(2,np.array([6,8])))
obstacles.append(factory.createCircle(2,np.array([13,2])))
obstacles.append(factory.createCircle(2,np.array([25,13])))
obstacles.append(factory.createCircle(2,np.array([2,4])))
print(obstacles)
mpc_controller.set_start_configuration(initial_state)
mpc_controller.set_obstacles(obstacles)

# DEFINE MPC HORIZON
mpc_controller.set_n_steps(6)
mpc_controller.set_time_horizon(6)

# DEFINE MPC OPTIMIZATION PENALTIES
mpc_controller.set_control_penalty(np.eye(2)*1E1)
mpc_controller.set_state_penalty(np.eye(3)*4E1)

# DEFINE CAR MODEL
mpc_controller.set_car_baseline(0.5)          #m
mpc_controller.set_max_yaw_rate(25*np.pi/180) #rad/s
mpc_controller.set_max_steering(20*np.pi/180) #rad
mpc_controller.set_max_speed(3)               #m/s
mpc_controller.set_min_speed(-0.5)              #m/s
mpc_controller.init_controller()


# target_state = np.array([[1,1,0],
#                          [4,1.5,10*np.pi/180],
#                          [6,2,20*np.pi/180],
#                          [8,4,90*np.pi/180],
#                          [5,6,180*np.pi/180],
#                          [3,6,160*np.pi/180],
#                          [2,8,90*np.pi/180],
#                          [4,9,0*np.pi/180],
#                          [7,8,-10*np.pi/180],
#                          [10,6,-45*np.pi/180],
#                          [12,2,-45*np.pi/180]]) 

# target_state = np.array([[1,1,0],
#                          [2,2,0*np.pi/180],
#                          [5,2,0*np.pi/180],
#                          [8.5,1.8,-5*np.pi/180],
#                          [10,3,70*np.pi/180],
#                          [9,6.5,180*np.pi/180],
#                          [5.5,6.5,185*np.pi/180],
#                          [4.,7,90*np.pi/180],
#                          [4.7,8,0*np.pi/180],
#                          [6,7.5,0*np.pi/180],
#                          [9,8.5,0*np.pi/180],
#                          [11,8,20*np.pi/180],
#                          [12,8.5,0*np.pi/180],
#                          [13,8.3,-35*np.pi/180],
#                          [14,7.3,0*np.pi/180],
#                          [16,8.9,0*np.pi/180],
#                          [17,7.5,-70*np.pi/180],
#                          [15,4,-98*np.pi/180],
#                          [14.5,2.5,-90*np.pi/180],
#                          [16,1.7,-5*np.pi/180]]) 

#target_state[:,:2] =30*target_state[:,:2]

fig=plt.figure()
ax = plt.gca()

for obstacle in obstacles :
    circle = plt.Circle(obstacle['center'], obstacle['radius'], color='r')
    ax.add_patch(circle)

for ii in range(len(target_state)) :
    ax.scatter(target_state[ii,0],target_state[ii,1],s=100)


def switch(state,target,mindist):
    dist=np.sqrt((state[0]-target[0])**2+(state[1]-target[1])**2)
    return dist<=mindist

counter = 0
for ii in range(200) :
     print(mpc_controller.state)
     target = target_state[counter,:]
     print(target)
     u = mpc_controller.mpc_optimal_control(target[:,np.newaxis])
     ax.plot(mpc_controller.state[0],mpc_controller.state[1],'*',linewidth=10)
     
     mpc_controller.step(u)
     
     plt.pause(0.05)
     if switch(mpc_controller.state,target,0.5):
              counter = counter+1
              if counter+1 >len(target_state) :
                  print('Thanks for the ride!')
                  break
plt.show()

# counter = 0
# while running:
#     # check if quit was pressed
#     for event in pygame.event.get():
#         if event.type == pygame.QUIT:
#            running = False
#            pygame.quit()
#            sys.exit()
    
#     # reset the image  
#     screen.fill((0, 0, 0))
#     if switch(mpc_controller.state,target_state[counter,:],0.5):
#                   counter = counter+1
#     pygame.draw.circle(screen,(250,250,0),(initial_state[0],initial_state[1]),10)
#     pygame.draw.circle(screen,(250,250,0),(target_state[counter,0],target_state[counter,1]),10)
#     u = mpc_controller.mpc_optimal_control(target_state[counter,:][:,np.newaxis])
#     mpc_controller.step(u)
#     mpc_controller.draw_predicted_state(screen)
    
#     pygame.display.flip()    
  