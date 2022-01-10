import pygame
import sys,os
import numpy as np
import random
from   random import randint
from scipy.spatial.kdtree import minkowski_distance
from   casadi import *
import matplotlib.pyplot as plt
import time

if __name__ == '__main__' :
     import CollisionChecks
     import ObstaclesFactory
     from Dubin import Dubin
     
else :
    
     from . import CollisionChecks
     from . import ObstaclesFactory
     from .Dubin import Dubin

class EnvMap():
    def __init__(self, start, goal, MapDimensions) :
        
        self.start           = start   #(x,y,theta)   configuration
        self.goal            = goal    #(x,y,theta)   configuration
        self.MapDimensions   = MapDimensions
        self.Mapw, self.Maph = self.MapDimensions

        # Window settings
        self.MapWindowName = "RRT Path Planning"
        pygame.display.set_caption(self.MapWindowName)
        self.map = pygame.display.set_mode((self.Mapw, self.Maph))
        self.map.fill((255, 255, 255))
        self.nodeRad       = 2
        self.nodeThickness = 0
        self.edgeThickness = 1

        self.obstacles = []

        # Colours
        self.grey = (70, 70, 70)
        self.blue = (0, 0, 255)
        self.green = (0, 255, 0)
        self.red = (255, 0, 0)
        self.white = (255, 255, 255)
        
        
      # line thickness
        
        self.nodeRad = 2        
        self.nodeThickness = 0 
        self.edgeThickness = 1
    

   
    def get_obstacles(self):
        return self.obstacle_list
    
        
    def add_obstacles(self,new_list_of_obstacles) :
        # ----------DESCRIPTION--------------
        # add obstacles to current obstacles list
        # -----------INPUT-------------------
        #  new_list_of_obstacles     ---> list of dictionaries
        # -----------OUTPUT------------------
        # update state 
        
        self.obstacles=new_list_of_obstacles
    
    def reset_obstacles(self,new_list_of_obstacles,robot_radius) :
        # ----------DESCRIPTION--------------
        # reset obstacles
        # -----------INPUT-------------------
        #  new_list_of_obstacles     ---> list of dictionaries

        # -----------OUTPUT------------------
        # update state 
        
        self.obstacle_list=new_list_of_obstacles
    
    def check_feasibility(self,robot_radius=0):
        # eliminated the obstacles that are 
        # exactly on the goal or the start position
        # retruns a list to update the EnvMap class
        
        # for now no angle checks . we assume
        # every possible tilted orientation 
        # at a given position will be fine
        obs   = self.obstacles.copy()
        start = np.asarray(self.start[:2],dtype=np.float32)
        goal  = np.asarray(self.goal[:2],dtype=np.float32)

        for n,obstacle in enumerate(obs) :
            
            center    = np.asarray(obstacle["center"],np.float64)

            dist2start    = np.sqrt(np.sum((center-start)**2))
            dist2goal     = np.sqrt(np.sum((center-goal)**2))
            
            check1        = dist2start<robot_radius+obstacle['radius']
            check2        = dist2goal<robot_radius+obstacle['radius']
            

            #safety limit of three radius
            if check1  or check2   :
                self.obstacles.pop(n)
        
        return self.obstacles
        
        
    def get_centers(self):
        # ----------DESCRIPTION--------------
        # obtain obsracles centers
        # -----------INPUT-------------------
        # NONE
        # -----------OUTPUT------------------
        # centers                     ---> list of tuple
        
        centers = [tuple(obstacle['centers']) for obstacle in self.obstacle_list ]
        return centers 
     
    def draw_startgoal(self):
        # to be modified if you change the 
        # start goal definition
        pygame.draw.circle(self.map,self.green,self.start[:2],4)
        pygame.draw.circle(self.map,self.green,self.goal[:2],4)
        
        
    def draw_obstacles(self) :
        
        # ----------DESCRIPTION--------------
        # draw obstacles in the screen
        # -----------INPUT-------------------
        # screen             ----> pygame.screen object
        # -----------OUTPUT------------------

        
        for obstacle in self.obstacles :
            if obstacle['type'] == 'circle' :
                pygame.draw.circle(self.map,obstacle['color'],tuple(obstacle['center']),obstacle['radius'])
            else :
                pygame.draw.polygon(self.map,obstacle['color'],tuple(obstacle['vertices']))
    
    
    def drawPath(self, path):
        path = np.array(path)
        pygame.draw.lines(self.map, self.red,False,path[:,:2],5)
    
    @classmethod
    def rot2d(cls,vertices, angle):
        
        # takes a list of 2d vertices and rotates them
        # vertices are always 2d arrays with a vertex for each line
        
        mat = np.array([[np.cos(angle), -np.sin(angle)],
                        [np.sin(angle), np.cos(angle)]])
        new = mat @ vertices.T
        return new.T
    
    
    def createRandomMap(self,nObs,type,minRadius=20,maxRadius = 50,maxFaces = 6):
        # ----------DESCRIPTION--------------
        # creates set of random obstacles
        # of a fiven type
        # -----------INPUT-------------------
        #  nObs     number of obstacles    ---> float
        #  type     type of obstacles      ---> list of strings
        #  maxRadius max bounding radius   ---> float
        #  minRadius min bounding radius   ---> float
        #  maxFaces  max faces per polygon ---> int
        # 
        # 
        # example : ['circle','rectangle','polygon']
        #  note --> for now only collision check with polygons and 
        #           rectangles. TODO 
        
        
        # -----------OUTPUT------------------
        # update state self.obstcle_list
        
        # specify bounds on the obstacles
        maxFaces = int(maxFaces) 
        nObs     = int(nObs)
        obs_list = []
        for n in range(nObs) :
            
            obstype  = random.choice(type)
            rad      = random.randrange(minRadius,maxRadius)
            center   = np.array([random.randint(0,self.Mapw),random.randint(0,self.Maph)])
            
            if obstype == 'polygon' :
                nfaces   = random.randint(3,maxFaces) 
                obstacle = ObstaclesFactory.createNpolygon(nfaces,rad,center)
                
            elif obstype == 'rectangle' :
                width    = random.randint(minRadius,maxRadius) 
                height   = random.randint(minRadius,maxRadius)
                obstacle = ObstaclesFactory.createRectangle(width,height,center)
            
            obs_list.append(obstacle)
        
        self.obstacle_list = obs_list
        return obs_list

    def load_room_map(self, filename):

        self.data = []
        with open(filename, 'rt') as f:
            for line in f:
                self.data.append(line.strip())

        width = self.Mapw/len(self.data[0])
        height = self.Maph / len(self.data)

        obs_list = []

        for row, tiles in enumerate(self.data):
            for col, tile in enumerate(tiles):
                if tile == '1':
                    center = np.array([(col)*width + width/2, (row)*height+height/2])
                    obstacle = ObstaclesFactory.createRectangle(width, height, center)

                    obs_list.append(obstacle)

        self.obstacle_list = obs_list
        return obs_list
    
    
    
    
## MPC controller class

class Controller_mpc :
    
    def __init__(self) :
        pass
    
    
           
        
        
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
    def __init__(self,x=0,y=0,yaw=0,dimension=(30,15)) :
        
        # state of the system
        self.x      = x    # m     x of the baricenter
        self.y      = y    # m     y of the baricenter
        self.yaw    = yaw   # rad   angle of the car with the global frame (0 = aligned with x-axis)
        
                             #       Combined w/ velocity it causes yaw rate
        self.state     = [self.x,self.y,self.yaw]
        
        # obtaine robot vertices
        rect          = ObstaclesFactory.createRectangle(dimension[0],dimension[1])       
        self.vertices = rect['vertices']  # list of vertices of the rectangle
        self.radius   = rect['radius']    # bounding radius
        self.L        = dimension[0]      # m   length of wheels base line
    
    
    def initialise_sprite(self,image_file) :
        
        self.image_file = image_file
        self.image_original = pygame.image.load(self.image_file).convert()
        self.image_original = pygame.transform.scale(self.image_original, (85, 65)) 
        self.rect_original  = self.image_original.get_rect()
        self.rect_original.center = (self.x,self.y)
        self.image                = pygame.transform.scale(self.image_original, (85, 65)) 
        
    def set_car_spec(self,vel_max=1,max_yaw_rate=10):
        """set car max yawrate and speed"""
        self.vmax   = vel_max
        self.maxyaw = max_yaw_rate
        self.maxphi         = np.arctan(self.maxyaw*self.L /self.vmax)
    
    def set_baseline(self,baseline = 10):
        self.L = baseline
        
    def set_obstacles(self,obstacles_list=None) :
        # define obstacles dictionary with bounding radius
        # input :
        #
        # obstacles_list : list of dictionaries containing all the obstacles definitions
        #                  see obstacles factory for a full description 
        #                  of the obstacle dictionary definition
        
        if len(obstacles_list) == 0 :
           self.obstacles = [obstacles_list]
           
        else :
            self.obstacles = obstacles_list
    
    # def car_step(self,u) :
    #     """takes the control speed and phi and returns new state of the system""" 
        
    #     v   = u[0]
    #     phi = u[1]
        
    #     self.x += v*np.cos([self.yaw])*self.dt
    #     self.y += v*np.sin([self.yaw])*self.dt
    #     self.yaw += v/self.L*np.tan(phi)*self.dt
         
        
    def step(self,control,dt) :
        """ updates the state of the system given the control input 
        # should be given
        
         Input 
        
           control       :  np.array([v,phi])
        
                        v       --> speed          [m/s]
                        phi     --> steering angle [rad/s]
           
           dt           :actuation time
                        
        """
        # define system variables
        x     = MX.sym("x")
        y     = MX.sym("y")
        theta = MX.sym("theta")
        
        #define control input 
        v     = MX.sym("v")
        phi   = MX.sym("phi")
        
        # define dynamics function
        # the system is non linear so we need to define an integration
        # function so that state_next = int(f(state,u),t0,t1)
        # FOR LINEAR SYSTEMS IT IS EASY. FOR NON LINEAR ONES IT IS NOT

        state = hcat([x,y,theta])
        u     = hcat([v,phi])
        
        ode = {"x"   : state,
                "p"   : u,
                "ode" : hcat([u[0]*cos(state[2]),u[0]*sin(state[2]),u[0]/self.L*tan(u[1])])}

        # define integration options
        options = {"tf"                        : dt, 
                    "simplify"                 : True,
                    "number_of_finite_elements": 4}

        # define state transition function
        Phi    = integrator("F","rk", ode,options) #define integration step
        res    = Phi(x0 = state ,p=u)              #define symbolic output
        x_next = res["xf"]                         # define symbolic output expression
        
        #define single input/output function
        Phi = Function(  'Phi'  ,[state,u]   ,[x_next]    ,['x','u']     ,['x_next']) 
        
        self.step_function = Phi
        
        # set optimiser for mcp control loop
        # optimizer
        # define optimization cost
        
        
        next_state = self.step_function(self.state,control)
        self.state = next_state
    
        self.x=self.state[0,0]
        self.y=self.state[1,0]
        self.yaw=self.state[2,0]
        
    def update_sprite(self) :
        
        gray                = (247,247,247) 
        self.image          = pygame.transform.scale(self.image_original, (85, 65)) 
        
        
        self.image  = pygame.transform.rotate(self.image_original, -90-self.yaw*180/np.pi)
        
        self.rect           = self.image.get_rect()
        self.rect.center    = (self.x,self.y) # save unrotated image center
        self.image.set_colorkey(gray)

    
    def reset_state(self,state=(0,0,0)):
        self.x,self.y,self.yaw= state  
        self.state            = state   
    
    def get_current_vertices(self):
        pos      = np.array([self.x,self.y])
        vertices = pos+EnvMap.rot2d(self.vertices,self.yaw)
        
        return vertices        
    #  here the equations of motion are defined
    
    def get_current_position(self):
        return np.array([self.x,self.y])
    
    # draw the robot on the screen
    def draw_robot(self,screen,color=(0,250,0)):
        # ----------DESCRIPTION--------------
        # draw robot state on the screen
        # -----------INPUT-------------------
        # screen             ----> pygame.screen object
        # -----------OUTPUT------------------
        # updates state of the system
        pos      = np.array([self.x,self.y])
        vertices = pos+EnvMap.rot2d(self.vertices,self.yaw)
        
        pygame.draw.polygon(screen,color,vertices)
    

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
            robot_pos  = (self.x,self.y)
            pygame.draw.line(screen,color,(self.x,self.y),center)
            




class RRTplanner:
    def __init__(self,start,goal,MapDimensions,obsList,maxstep,robot):
        
        self.start           = start   #configuration
        self.goal            = goal    #configuration
        self.goalFlag        = False   # a path is found
        self.alreadyFound    = False   # a path already exist
        self.mapw, self.maph = MapDimensions
        self.limits          = (self.mapw,self.maph)
        self.maxstep         = maxstep
        
        # initialise tree
        self.nodes  = [start]  # List of tuples representing C space (x, y, theta)
        self.parent = [0]      # List of ints, representing each node's parent (ie node N's parent can be found at index N)
        self.cost   = [0]
        
        self.local_best_path   = []   # saves the local path between two nodes 
        self.parent_path       = [[]] # strores all the paths from parent to child node
        self.parent_actions    = [[]] # stores actions needed to follow the path
        self.actuation_time    = [0]
        
        
        self.current_best_cost = None # best cost to path
        self.start2goal_paths_lists = []
        self.number_of_path_found   = 0
        self.rebase_tree            = False   #activate rebasing 
        self.path_resolution        = 20      # number of points for each dubin path
        self.time_elapsed           = None    #total elapsed time 
        self.map_name               = 'Undefined'
        
        # obstacles
        self.obstacles = obsList # list of obstacles follwing the obstacles definitionnisnide ObstacleFactory
        
        # path
        self.goalstate = None
        self.path      = []
        
        
        # Now the robot class is given to the 
        # path planner so that it can use 
        # the robot definition to plan the path
         
        self.robot = robot # robot instanciation
    
    
    def set_map_name(self,name='Undefined'):
        """set map name"""
        self.map_name = name
    
    
    def set_path_resolution(self,res):
        """set resolution (number of points) for each dubin path"""
        
        self.path_resolution        = 20
    
    
    def activate_rebasing(self,rebase_gamma) :
        
        """activates rebasing option
        
        When turned on, each new point is connected with a dubin path to the 
        node which has the lower prent cost and not the closer one.
        The rebase search is given by 
        
        Parameters
        ----------
        
        rebase_rad = max(self.gamma*(np.log(self.numberOfNodes())/self.numberOfNodes())**(1/4),self.maxstep*2)
        
        """
        
        self.gamma  = rebase_gamma
        self.rebase_tree = True
        
    def start_clock(self):
        self.t_start = time.time() 
    
    def stop_clock(self) :
        self.time_elapsed = time.time()-self.t_start 
         
    def save_optimal_path(self,path,length,actions,time=None):
        """append optimal path to the current list of optimal paths"""
        self.start2goal_paths_lists.append({'path':path,'length':length,'time':time,'actions':actions})
    
    def get_number_of_paths_found(self):
        return self.number_of_path_found
        
    def addCost(self,n,cost) :
        """add cost to node"""
        self.cost.insert(n,cost)
    
    def removeCost(self,n):
        """remove cost"""
        self.cost.pop(n)
    
    def addNode(self, n, node):
        """add node in a given index"""
        self.nodes.insert(n, node)


    def removeNode(self, n):
        """remove a node from a given index"""
        # remove a node at a given
        # index n
        self.nodes.pop(n)
    
    
    def addPath(self,n):
        """add the local best path from prent to child"""
        self.parent_path.insert(n,self.local_best_path) # save only the position and not the angle
        self.parent_actions.insert(n,self.local_best_actions)
        self.actuation_time.insert(n,self.local_best_dist/self.robot.vmax/self.path_resolution)

  
        
    def removePath(self,n):
        """add the local best path from prent to child"""
        self.parent_path.pop(n) # save only the position and not the angle
        
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
    
    
    def euclide_distance(self,n1,n2) :
        node1 = np.asarray(self.nodes[n1],dtype=np.float64)
        node2 = np.asarray(self.nodes[n2],dtype=np.float64)

        return np.linalg.norm(node1[:2]-node2[:2])
    
    def dubin_distance(self, n1, n2):
        """find distance betweeen two points"""
        pathfinder         = Dubin(self.nodes[n1],self.nodes[n2][:2],robot_obj=self.robot,n_samples=self.path_resolution) #Rturn=self.robot.vmax/self.robot.maxyaw
        paths,pathlengths,actions  = pathfinder.make_path()
       
        # take only bets options
        minDistance        = pathlengths[0] # take the shortest path
        currentpath        = paths[0]
        best_actions       = actions[0]
        
        
        return minDistance,currentpath,best_actions
    
    def sample_envir(self):
        #sample position from the map 
        randnode = (random.uniform(0,lim) for lim in self.limits )
        return randnode

    def dubin_nearest(self, n):
        """find closest node to the final node inside a certain radius"""
        
        if self.rebase_tree : 
             rebase_rad      = max(self.gamma*(np.log(self.numberOfNodes())/self.numberOfNodes())**(1/4),self.maxstep*2)
        else :
             rebase_rad     = self.maxstep*2
        
        promising_nodes = []
        
        for i in range(0, n):
            dist = self.euclide_distance(i, n)
            if dist <rebase_rad :
                promising_nodes.append(i) 
        
        if len(promising_nodes)==0 :
            raise ValueError('search radius to small\nNo solution found inside the search radius')
        
        nnear = promising_nodes[0]
        guess = promising_nodes.pop(0)
        best_dubin_dist,best_path,best_actions = self.dubin_distance(guess, n)
        
        if self.rebase_tree :
            min_cost = self.cost[guess] +  best_dubin_dist
        else :
            min_cost =  best_dubin_dist
            
        
        self.local_best_path    = best_path
        self.local_best_actions = best_actions
        self.local_best_dist    = best_dubin_dist
        
        for good_node in promising_nodes:
            
            new_dubin_dist,new_path,new_actions = self.dubin_distance(good_node, n)
            
            if self.rebase_tree :
                
                new_min_cost = self.cost[good_node] + new_dubin_dist
            
            else :
                new_min_cost = new_dubin_dist
                
                
            if new_min_cost<min_cost :
                best_dubin_dist      = new_dubin_dist # best local distance
                
                self.local_best_path    = new_path
                self.local_best_actions = new_actions
                self.local_best_dist    = new_dubin_dist
                
                nnear                = good_node
                min_cost             = new_min_cost
        
        return nnear,best_dubin_dist,best_actions
    
    def euclidean_nearest(self, n):
        """find distance of all nodes from the last node in the list i.e n"""
        dmin = self.euclide_distance(0, n)# to be changed in case of RRT star
        nnear = 0
        
        for i in range(0, n): # all except the node itself
            if self.euclide_distance(i, n) < dmin:
                dmin = self.euclide_distance(i, n)
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
            
            self.robot.reset_state((x,y,theta))
            # hierchical checking
            # 1) circle collsion
            # 2) GJK finer collision checks 
            
            if CollisionChecks.CircleCollision(obstacle['radius'],self.robot.radius,obstacle['center'],self.robot.get_current_position()) :
                if CollisionChecks.GJK(self.robot.get_current_vertices(),obs_vert): 
                   self.removeNode(n)
                   return False
        return True

    def crossObstacle(self):
        
        # retruns :
        # true/false
        # final state from dubins 
        
        final_state = self.local_best_path[-1,:]
        obs         = self.obstacles.copy()
        while len(obs) > 0:
            obstacle   = obs.pop(0)
            obs_vert   = obstacle['vertices']
            
            for i in range(len(self.local_best_path[:,0])):
                x = self.local_best_path[i,0]
                y = self.local_best_path[i,1]
                theta = self.local_best_path[i,2]
                # hirerchical checking
                # 1) circle collsion
                # 2) GJK finer collision checks
                self.robot.reset_state((x,y,theta))
                # hirerchical checking
                # 1) circle collsion
                # 2) GJK finer collision checks
                if CollisionChecks.CircleCollision(obstacle['radius'],self.robot.radius,obstacle['center'],self.robot.get_current_position()) :
                   if CollisionChecks.GJK(self.robot.get_current_vertices(),obs_vert): 
                      return True,final_state
        
        
        return False,final_state

    def connect(self, n1, n2,best_dubin_dist):
        """Connect two nodes, (only if possible)"""
        
        iscrossing,final_state=self.crossObstacle()
        
        if abs(self.local_best_path[-1,0] - self.goal[0]) < self.maxstep/2 and abs(self.local_best_path[-1,1] - self.goal[1]) < self.maxstep/2:
                self.goalFlag=True
                
        
           
        if iscrossing:
            self.removeNode(n2)
            self.local_best_path=[]
            self.local_best_actions = []
            if self.goalFlag :
                self.goalFlag = False
                return False

        else:
            
         
            
            self.nodes[n2] = final_state # put the real final state as computed from dubins path
            self.addEdge(n1, n2)
            self.addPath(n2)
            self.addCost(n2,self.cost[n1]+best_dubin_dist)
            
            
            if self.goalFlag and (not self.alreadyFound) :
       
                self.goalstate = n2
                self.current_best_cost = self.cost[n2]
                self.goalFlag  = False
                self.alreadyFound = True
                self.number_of_path_found   += 1
                
            elif self.goalFlag and self.alreadyFound :
                
                if self.current_best_cost > self.cost[n2] :
                
                    self.goalstate = n2
                    self.current_best_cost = self.cost[n2]
                    self.goalFlag  = False
                    self.number_of_path_found   += 1
                else :
                    
                    self.goalFlag  = False
                
        return True
 
    def step(self, nnear, nrand):
        # takes a step in a random dierection
        # if the new node is too distant from
        # the nearest node, a smaller step
        # is taken with dubin_distance dmax
        
        d = self.euclide_distance(nnear, nrand)
        # definition of distance max is crucial
        
        if d > self.maxstep :
            # take a straight line in the direction
            # of the far point which is as big
            # as the maximum allowed distance
            
            xnear, ynear = self.nodes[nnear][0], self.nodes[nnear][1]
            xrand, yrand = self.nodes[nrand][0], self.nodes[nrand][1]
            
            (px, py)     = (xrand - xnear, yrand - ynear)
            theta        = np.arctan2(py, px)
            (x, y)       = (xnear + self.maxstep*np.cos(theta), ynear + self.maxstep*np.sin(theta))
            
            self.removeNode(nrand)
            self.addNode(nrand, (x, y,0)) # the actual theta is calculated 
                                              # by the dubin path later
    
    def isGoalReached(self):
        return self.goalFlag
    
    def isOnePathFound(self):
        return self.alreadyFound

    def getFinalPath(self):
        path_resoultion =  len(self.parent_path[1][:,0])
        
        if self.alreadyFound:
            self.best_final_path       = np.flipud(self.parent_path[self.goalstate][:path_resoultion,:]).tolist() # the last node is always eliminated because it is equal to start of the following path
            self.best_final_actions    = np.flipud(self.parent_actions[self.goalstate][:path_resoultion-1,:]).tolist() # the last node is always eliminated because it is equal to start of the following path
            self.best_final_path_nodes = [self.goalstate]
            self.best_actuation_time   = [self.actuation_time[self.goalstate]]*self.path_resolution
            
            
            totalcost = self.cost[self.goalstate]
            parent    = self.parent[self.goalstate]
            
            while (parent != 0):
                self.best_final_path+=(np.flipud(self.parent_path[parent][:path_resoultion-1,:]).tolist())
                self.best_final_actions+=(np.flipud(self.parent_actions[parent][:path_resoultion-1,:]).tolist())
                self.best_final_path_nodes.append(parent) #except starting zero 
                self.best_actuation_time+=[self.actuation_time[parent]]*self.path_resolution
                parent = self.parent[parent]
            
            # save the actions as (v,phi,dt)
            
            self.best_final_actions = [[action[0],action[1],dt] for action,dt in zip(self.best_final_actions,self.best_actuation_time)]
            

        return self.best_final_path,self.best_final_actions,totalcost
    
    # NOT IMPLEMENTED ANYMORE
    
    # def bias(self, ngoal):
    #     """Take step directly towards the goal"""
    #     n = self.numberOfNodes()
    #     self.addNode(n, (ngoal[0], ngoal[1], ngoal[2]))
       
    
        
    #     if self.isFree():
    #         xnearest = self.euclidean_nearest(n)
    #         self.step(xnearest, n)
    #         xnearest_dubin,best_dubin_dist = self.dubin_nearest(n) #sets directly the best_path to the nearest point
    #         self.connect(xnearest_dubin, n,best_dubin_dist)
            
    #     return self.nodes, self.parent

    def expand(self):
        """Take random step"""
        n = self.numberOfNodes()
        x, y = self.sample_envir()
        self.addNode(n, (x, y, 0))  # !! theta randomly set to 0, as it is currently ignored. To be calculated by Dubin
        
        if self.isFree():
            xnearest = self.euclidean_nearest(n)
            self.step(xnearest, n)
            xnearest_dubin,best_dubin_dist,best_actions = self.dubin_nearest(n) #sets directly the best_path to the nearest point
            self.connect(xnearest_dubin, n,best_dubin_dist)
            
        return self.nodes, self.parent


    def write_summary2txt(self,filename) :
        
        f = open(filename,'w')
        
        ## intro 
        f.write('----------------------------'+'\n')
        f.write('RRT search Summary          '+'\n')
        f.write('----------------------------'+'\n')
        f.write('Map Name               : {}'.format(self.number_of_path_found)+'\n')
        f.write('Total paths found      : {}'.format(self.number_of_path_found)+'\n')
        f.write('Total number of nodes  : {}'.format(self.numberOfNodes())+'\n')
        f.write('Total elapsed time     : {} s'.format(self.time_elapsed)+'\n')
        if self.rebase_tree:
            f.write('Solver                 : {} '.format('RRTstar')+'\n')
        else : 
            f.write('Solver                 : {} '.format('RRT standard')+'\n')
        
        if len(self.start2goal_paths_lists) >0 : 
            for pathspec in  self.start2goal_paths_lists :
                
                counter = 0
                path   = pathspec['path']
                length = pathspec['length']
                time   = pathspec['time']
                actions   = pathspec['actions']
                f.write('----------------------------'+'\n')
                f.write('Path number   : {}'.format(counter)+'\n')
                f.write('Path length   : {}'.format(length)+'\n')
                f.write('Time required : {} s'.format(time)+'\n')
                f.write('>>>> Nodes'+'\n')
                for cord in path :
                    f.write(str(cord)+'\n')
                f.write('>>>> '+'\n')
                f.write('>>>> inputs'+'\n')
                
                for cord in actions :
                    f.write(str(cord)+'\n')
                f.write('>>>> '+'\n')
                
                counter = counter +1
        else :
            
            f.write('----------------------------'+'\n')
            f.write('NO PATH TO GOAL FOUND\n')
            
        f.close()


def readPathDataFromTxt(filepath):
    
    f       = open(filepath,'r')
    lines   =  f.readlines()
    counter = 0

    save_output = [] # list of dictionaries
    for line in lines[8:] :
        
        if line.strip() =='NO PATH TO GOAL FOUND' :
            save_output = {'path_number': None, 'length':None,'time':None}
            
        line = line.split(':')
        if len(line)>1 :
            if "Path number"==line[0].strip() :
                save_output.insert(counter,{'path_number': float(line[1])})
            if "Path length"==line[0].strip() :
                save_output[counter]['length']=float(line[1])
            if "Time required"==line[0].strip() :
                save_output[counter]['time']=float(line[1].split('s')[0])
                counter += 1
                
        f.close()
        
    return save_output

results = readPathDataFromTxt("./OutputSimulations/Simulation0.txt")
