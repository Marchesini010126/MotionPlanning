import pygame
import sys,os
import numpy as np
import random
from   random import randint
from scipy.spatial.kdtree import minkowski_distance
from   casadi import *
import matplotlib.pyplot as plt

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
    
    def reset_obstacles(self,new_list_of_obstacles) :
        # ----------DESCRIPTION--------------
        # reset obstacles
        # -----------INPUT-------------------
        #  new_list_of_obstacles     ---> list of dictionaries
        # -----------OUTPUT------------------
        # update state 
        
        self.obstacle_list=new_list_of_obstacles
    
    def check_feasibility(self):
        # eliminated the obstacles that are 
        # exactly on the goal or the start position
        # retruns a list to update the EnvMap class
        
        # for now no angle checks . we assume
        # every possible tilted orientation 
        # at a given position will be fine
        obs = self.obstacles.copy()
        start = np.asarray(self.start[:2],dtype=np.float32)
        goal  = np.asarray(self.goal[:2],dtype=np.float32)

        for n,obstacle in enumerate(obs) :
            
            check1    = np.sum((obstacle['center']-start)**2)
            check2    = np.sum((obstacle['center']-goal)**2)
            #safety limit of three radius
            if check1< (4*obstacle['radius'])**2 or check2< (4*obstacle['radius'])**2 :
                obs.pop(n)
        
        self.obstacles = obs
        
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
        pygame.draw.lines(self.map, self.red,False,path[:,:2],2)
    
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
        self.vx     = 0.0   # m/s
        self.yaw    = 0.0   # rad   angle of the car with the global frame (0 = aligned with x-axis)
        self.phi    = 0.0   # rad   angle of the wheels with local x-axis (0 = forward, negative = left).
                             #       Combined w/ velocity it causes yaw rate
        self.state   = np.array([self.x,self.y,self.yaw])
         
        # dynamic parameters
        self.dt        = 0.01 # s         # integration step
        self.mass      = 10   # kg        # mass
        self.max_phi   = np.pi / 8        # rad   max steering angle of the wheels
        
        # obtaine robot vertices
        rect          = ObstaclesFactory.createRectangle(dimension[0],dimension[1])       
        self.vertices = rect['vertices']  # list of vertices of the rectangle
        self.radius   = rect['radius']    # bounding radius
        
        self.L        = dimension[0]      # m   length of wheel base
    
    
    def set_time_horizon(self,Ts=10):
        # horizon of the MPC controller
        self.T = Ts
    def set_n_steps(self,nsteps=20):
        #define number of the steps in the time horison
        self.n_steps = nsteps
    def set_car_baseline(self,L=0.30):
        #define car baseline
        self.L   = L
    def set_max_speed(self,vmax=1):
        self.vmax=vmax
    def set_min_speed(self,vmin=0):
            self.vmin=vmin
    def set_max_steering(self,phi=1):
        # in radians
            self.phi_max=phi
    def set_max_yaw_rate(self,maxyaw):
        # in radians/s
            self.maxyaw=maxyaw  
    def set_control_penalty(self,Q=np.eye(2)):
        # define control penalty matrix
        # for the mpc definition
        #  sum(x.T R x + u.T Q u)
        
        self.Q = np.sqrt(Q)
        # note -> the root square is taken because of 
        # the optimization problem formulation
    
    def set_state_penalty(self,R=np.eye(2)):
        # define state penalty matrix
        # for the mpc definition
        # sum(x.T R x + u.T Q u)
        
        self.R = np.sqrt(R)
        # note -> the root square is taken because of 
        # the optimization problem formulation
    
    '''to eliminate''' 
    
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
        
    def init_car(self):
        
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

        state = vcat([x,y,theta])
        u     = vcat([v,phi])
        
        ode = {"x"   : state,
                "p"   : u,
                "ode" : vcat([u[0]*cos(state[2]),u[0]*sin(state[2]),u[0]/self.L*tan(u[1])])}

        # define integration options
        options = {"tf"                        : self.T/self.n_steps, 
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
        
    def step(self,control) :
        """ updates the state of the system given the control input 
        # should be given
        
         Input 
        
           control       :  np.array([v,phi])
        
                        v       --> speed          [m/s]
                        phi     --> steering angle [rad/s]
                        
        """
        
        next_state = self.step_function(self.state,control)
        self.state = np.array(next_state)
        self.x=self.state[0,0]
        self.y=self.state[1,0]
        self.yaw=self.state[2,0]
        
    
    
    def mpc_optimal_control_action(self,target_state) :
        # take an mpc step toward the target using the curret state
        ## Input 
        #
        # target_state :  np.array([x,y,theta])   initial state of the system
        #                  
        #                 x     --> x-position [m]
        #                 y     --> y-position [m]
        #                 theta --> heading [rad]
        
        self.optimser = casadi.Opti()

        # define the varibles
        x = self.optimser.variable(3,self.n_steps+1) # for n+1 steps
        u = self.optimser.variable(2,self.n_steps)   # for n_steps 
        
        obstacles_cost = 0.0
        # add kinematic constraints
        for ii in range(self.n_steps) :
            self.optimser.subject_to(x[:,ii+1]==self.step_function(x[:,ii],u[:,ii])) # system dynamics constraint  
            self.optimser.subject_to([-self.phi_max<=u[1,ii],u[1,ii]<= self.phi_max])
            self.optimser.subject_to([self.vmin<=u[0,ii],u[0,ii]<= self.vmax])
            self.optimser.subject_to([-self.maxyaw<=u[0,ii]/self.L*tan(u[1,ii]),self.maxyaw >=u[0,ii]/self.L*tan(u[1,ii])])
            try :
                for obstacle in self.obstacles :
                    center = obstacle['center'][:,np.newaxis]
                    distance = obstacle['radius'] + self.radius
                    self.optimser.subject_to((x[0,ii]-center[0])**2+(x[1,ii]-center[1])**2 > distance**2) # system dynamics constraint  
                    self.optimser.minimize(1/(x[0,ii]-center[0])**2+(x[1,ii]-center[1])**2)
            except  AttributeError :
                pass
        
        # initial state constraint
        self.optimser.subject_to(x[:,0] == self.state)   
        self.optimser.minimize(sumsqr(self.R@(x-target_state))+sumsqr(self.Q@u))
    

        #solve the problem
        #using interior point method 
        opts   = {'ipopt.print_level': 0, 'print_time': 0, 'ipopt.sb': 'yes'}
        s_opts = {'max_iter': 500}
        
        self.optimser.solver('ipopt',opts,s_opts)
        
        try :
           sol = self.optimser.solve()
        except :
           self.optimser.debug.show_infeasibilities()
           return
       
        control              = sol.value(u)[:,0]  # take only the first optimal control 
        self.predicted_state = sol.value(x)       # save predicted state
        
        return control
    
    
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
        
        # initialize tree
        self.nodes  = [start]  # List of tuples representing C space (x, y, theta)
        self.parent = [0]      # List of ints, representing each node's parent (ie node N's parent can be found at index N)
        self.cost   = [0]
        self.local_best_path = [] # saves the local path between two nodes 
        self.parent_path     = [[]] # strores all the paths from parent to child node
        self.current_best_cost = None # best cost to path
        self.start2goal_paths_lists = []
        self.number_of_path_found   = 0
        # obstacles
        self.obstacles = obsList
        # path
        self.goalstate = None
        self.path      = []
        
        
        # Now the robot class is given to the 
        # path planner so that it can use 
        # the robot definition to plan the path
         
        self.robot = robot # robot instanciation
        
    
    def save_optimal_path(self,path,length):
        
        self.start2goal_paths_lists.append({'path':path,'length':length})
    
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
        pathfinder         = Dubin(self.nodes[n1],self.nodes[n2][:2],Rturn=self.robot.vmax/self.robot.maxyaw)
        paths,pathlengths  = pathfinder.make_path()
        minDistance        = pathlengths[0] # take the shortest path
        
        # define local dubins path
        currentpath = paths[0]
        
        return minDistance,currentpath
    
    def sample_envir(self):
        #sample position from the map 
        randnode = (random.uniform(0,lim) for lim in self.limits )
        return randnode

    def dubin_nearest(self, n):
        """find closest node to the final node inside a certain radius"""
        rebase_rad = max(300*(np.log(self.numberOfNodes())/self.numberOfNodes())**(1/3),self.maxstep*2)
        
        
        
        promising_nodes = []
        
        for i in range(0, n):
            dist = self.euclide_distance(i, n)
            if dist <rebase_rad :
                promising_nodes.append(i) 
        
        if len(promising_nodes)==0 :
            raise ValueError('seach radius to small\nNo solution found inside the search radius')
        
        nnear = promising_nodes[0]
        guess = promising_nodes.pop(0)
        best_dubin_dist,best_path = self.dubin_distance(guess, n)
        
        min_cost = self.cost[guess] +  best_dubin_dist
        
        self.local_best_path = best_path
        
        for good_node in promising_nodes:
            
            new_dubin_dist,new_path = self.dubin_distance(good_node, n)
            new_min_cost = self.cost[good_node] + new_dubin_dist
            
            if new_min_cost<min_cost :
                best_dubin_dist      = new_dubin_dist # best local distance
                self.local_best_path = new_path
                nnear                = good_node
                min_cost             = new_min_cost
        return nnear,best_dubin_dist
    
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
            self.best_final_path=np.flipud(self.parent_path[self.goalstate][:path_resoultion-1,:]).tolist() # the last node is always eliminated because it is equal to start of the following path
            self.best_final_path_nodes = [self.goalstate]
            totalcost = self.cost[self.goalstate]
            parent    = self.parent[self.goalstate]
            while (parent != 0):
                self.best_final_path+=(np.flipud(self.parent_path[parent][:path_resoultion-1,:]).tolist())
                self.best_final_path_nodes.append(parent) #except starting zero 
                parent = self.parent[parent]
        
        return self.best_final_path,totalcost
    

    def rebase(self):
        if self.alreadyFound :
            rebase_rad = self.maxstep
            promising_nodes = []
            for node in self.best_final_path_nodes :
                  for node_from_map in range(0,self.numberOfNodes()):
                      if node_from_map != node :
                         dist = self.euclide_distance(node,node_from_map )
                         if dist <rebase_rad :
                            promising_nodes.append(node_from_map)
                
                  for good_node in promising_nodes: # check only first four
                
                    dubin_dist,path = self.dubin_distance(good_node,node)
                    if dubin_dist+self.cost[good_node]<self.cost[node] :
                        
                            best_dubin_dist      = dubin_dist
                            self.local_best_path = path
                            iscrossing,finalstate = self.crossObstacle()
                            if not iscrossing :
                               self.removeEdge(node)
                               self.removeCost(node)
                               self.removePath(node)
                               
                               self.addEdge(good_node,node)
                               self.addCost(node,self.cost[good_node]+best_dubin_dist)
                               self.addPath(node)
        return self.nodes,self.parent
                               
    def bias(self, ngoal):
        """Take step directly towards the goal"""
        n = self.numberOfNodes()
        self.addNode(n, (ngoal[0], ngoal[1], ngoal[2]))
       
    
        
        if self.isFree():
            xnearest = self.euclidean_nearest(n)
            self.step(xnearest, n)
            xnearest_dubin,best_dubin_dist = self.dubin_nearest(n) #sets directly the best_path to the nearest point
            self.connect(xnearest_dubin, n,best_dubin_dist)
            
        return self.nodes, self.parent

    def expand(self):
        """Take random step"""
        n = self.numberOfNodes()
        x, y = self.sample_envir()
        self.addNode(n, (x, y, 0))  # !! theta randomly set to 0, as it is currently ignored. To be calculated by Dubin
        
        if self.isFree():
            xnearest = self.euclidean_nearest(n)
            self.step(xnearest, n)
            xnearest_dubin,best_dubin_dist = self.dubin_nearest(n) #sets directly the best_path to the nearest point
            self.connect(xnearest_dubin, n,best_dubin_dist)
            
        return self.nodes, self.parent


    def writepath2txt(self,filename) :
        f = open(filename,'w')
        counter = 0
        for pathspec in  self.start2goal_paths_lists :
            path = pathspec['path']
            length = pathspec['length']
            f.write('path number : {}'.format(counter)+'\n')
            f.write('path length : {}'.format(length)+'\n')
            for cord in path :
                f.write(str(cord)+'\n')
            counter = counter +1
        f.close()
    