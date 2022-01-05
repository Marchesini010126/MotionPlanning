import pygame
import sys
import numpy as np
import random
from   scipy.spatial import distance
from   random import randint

from scipy.spatial.kdtree import minkowski_distance
import CollisionChecks
import ObstaclesFactory
import Dubin 

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
    def __init__(self,x0=0,y0=0,dimension=(30,15)) :
        
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
        
        # obtaine robot vertices
        rect          = ObstaclesFactory.createRectangle(dimension[0],dimension[1],center=np.array([self.x0,self.y0]))       
        self.vertices = rect['vertices']  # list of vertices of the rectangle
        self.radius   = rect['radius']    # bounding radius
        
        self.L        = 3                # m   length of wheel base
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
            



class RRTGraph:
    def __init__(self,start, goal,MapDimensions,obsList,RobotClass):
        
        self.start           = start
        self.goal            = goal
        self.goalFlag        = False
        self.maph, self.mapw = MapDimensions
        self.limits          = (self.mapw,self.maph)
        # initialize tree
        self.nodes  = [start]  # List of tuples representing C space (x, y, theta)
        self.parent = [0]      # List of ints, representing each node's parent (ie node N's parent can be found at index N)
        # obstacles
        self.obstacles = obsList
        # path
        self.goalstate = None
        self.path      = []

        
        # Now the robot class is given to the 
        # path planner so that it can use 
        # the robot definition to plan the path
         
        self.robot = RobotClass(self.start[0],self.start[1]) 
        

    def check_feasibility(self):
        # eliminated the obstacles that are 
        # exactly on the goal or the start position
        # retruns a list to update the EnvMap class
        
        # for now no angle checks . we assume
        # every possible tilted orientation 
        # at a given position will be fine
        
        start = np.asarray(self.start[:2],dtype=np.float32)
        goal  = np.asarray(self.goal[:2],dtype=np.float32)
        
        for n,obstacle in enumerate(self.obstacles) :
            
            check1    = np.linalg.norm(obstacle['center']-start)
            check2    = np.linalg.norm(obstacle['center']-goal)
            tolerance = 25
            if check1< tolerance + obstacle['radius'] or check2< tolerance+obstacle['radius'] :
                self.obstacles.pop(n)
        
        return self.obstacles
        

    def addNode(self, n, node):
        # add a node at a given
        # index n
        self.nodes.insert(n, node)


    def removeNode(self, n):
        # remove a node at a given
        # index n
        self.nodes.pop(n)
        

    def addEdge(self, parent, child):
        # add a parent node at a given
        # index n
        self.parent.insert(child, parent)

    def removeEdge(self, n):
        # remove a parent node at a given
        # index n
        self.parent.pop(n)

    def numberOfNodes(self):
        return len(self.nodes)
    
    
    
    def distance(self, n1, n2):
        """find distance betweeen two points"""
        pathfinder        = Dubin(self.nodes[n1],self.nodes[n2][:2])
        paths,pathlengths = pathfinder.make_paths()
        minDistance        = pathlengths[0] # take the shortest path
        
        return minDistance
    
    def sample_envir(self):
        #sample position from the map 
        randnode = (random.uniform(lim[1],lim[0]) for lim in self.limits )
        return randnode

    def nearest(self, n):
        dmin  = self.distance(0, n) 
        nnear = 0
        for i in range(0, n):
            if self.distance(i, n) < dmin:
                dmin = self.distance(i, n)
                nnear = i
        return nnear

        
    def isFree(self):
        """checks that the last node is free"""
        
        n            = self.numberOfNodes() - 1 # this is the last node index (python starts counting from zero)
        (x, y,theta) = self.nodes[n]
        obs          = self.obstacles.copy()
        
        while len(obs) > 0:
            obstacle   = obs.pop(0)
            obs_vert   = obstacle['vertices']
            # we need to check if the robots
            # can stay at a given position
            # with a given configuration
            
            robot_pos  = np.array([x,y])
            # hierchical checking
            # 1) circle collsion
            # 2) GJK finer collision checks 
            
            if CollisionChecks.CircleCollision(obstacle['radius'],self.robot.radius,obstacle['center'],robot_pos) :
                if CollisionChecks.GJK(self.robot.vertices,obs_vert): 
                   self.removeNode(n)
                   return False
        return True

    def crossObstacle(self,n1,n2):
        #n1 start node
        #n2 end   node
        
        node1, node2      = self.nodes[n1], self.nodes[n2]
        pathfinder        = Dubin(self.nodes[n1],self.nodes[n2][:2])
        paths,pathlengths = pathfinder.make_paths()
        
        path = [0] # obtain the shortest path array(Npoints,3)
        
        obs = self.obstacles.copy()
        while len(obs) > 0:
            obstacle   = obs.pop(0)
            obs_vert   = obstacle['vertices']
            
            for i in len(path[:,0]):
                x = path[i,0]
                y = path[i,1]
                # hirerchical checking
                # 1) circle collsion
                # 2) GJK finer collision checks
                robot_pos = np.array([x,y])
                # hirerchical checking
                # 1) circle collsion
                # 2) GJK finer collision checks
                if CollisionChecks.CircleCollision(obstacle['radius'],self.robot.radius,obstacle['center'],robot_pos) :
                   if CollisionChecks.GJK(robot_pos,obs_vert,radius=self.robot.radius): # the robot is a circle for now. So GJK is polygon vs Circle
                      return True
        return False

    def connect(self, n1, n2):
        """Connect two nodes, (only if possible)"""
        if self.crossObstacle(n1,n2):
            self.removeNode(n2)
            return False
        else:
            self.addEdge(n1, n2)
            return True

    def step(self, nnear, nrand, dmax=40):
        # takes a step in a random dierction
        # if the new node is too distant from
        # the nearest node, a smaller step
        # is taken with distance dmax
        
        d = self.distance(nnear, nrand)
        # definition of distance max is crucial
        
        if d > dmax:
            # take a straight line
            u            = dmax/d
            xnear, ynear = self.nodes[nnear][0], self.nodes[nnear][1]
            xrand, yrand = self.nodes[nrand][0], self.nodes[nrand][1]
            
            (px, py)     = (xrand - xnear, yrand - ynear)
            theta        = np.atan2(py, px)
            (x, y)       = (int(xnear + dmax*np.cos(theta)), int(ynear + dmax*np.sin(theta)))
            
            self.removeNode(nrand)
            if abs(x - self.goal[0]) < dmax and abs(y - self.goal[1]) < dmax:
                # if the goal is in the line add the goal 
                # as a final node
                self.addNode(nrand, (self.goal[0], self.goal[1], theta))
                self.goalstate = nrand
                self.goalFlag = True
            else:
                self.addNode(nrand, (x, y, theta))


    def pathToGoal(self):
        if self.goalFlag:
            self.path = []
            self.path.append(self.goalstate)
            newpos = self.parent[self.goalstate]
            while (newpos != 0):
                self.path.append(newpos)     #append the node to the list
                newpos = self.parent[newpos] #obtain the parent node
            self.path.append(0)
        return self.goalFlag

    def getPathCoords(self):
        return self.path
       

    def bias(self, ngoal):
        n = self.numberOfNodes()
        self.addNode(n, ngoal[0], ngoal[1])
        nnear = self.nearest(n)
        self.step(nnear, n)
        self.connect(nnear, n)
        return self.x, self.y, self.parent

    def expand(self):
        n    = self.numberOfNodes()
        node = self.sample_envir()
        self.addNode(n, node)       
        
        if self.isFree():
            xnearest = self.nearest(n)
            self.step(xnearest, n)
            self.connect(xnearest, n)
        return self.nodes, self.parent