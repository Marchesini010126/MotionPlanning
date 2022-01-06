import pygame
import sys
import numpy as np
import random
from   random import randint
from scipy.spatial.kdtree import minkowski_distance

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
        # path : list of tuple (x,y)
        file = 'path_nodes.txt'
        f=open(file,'w')
        for line in path :
            printer = ' '.join([str(a) for a in line])
            f.write(printer + '\n')
        f.close()
        
        
        pygame.draw.lines(self.map, self.red,False,path,2)
    
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
            center   = (random.randint(0,self.Mapw),random.randint(0,self.Maph))
            
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
        
        self.L        = 3                # m   length of wheel base
    
    def reset_state(self,state=(0,0,0)):
        self.x,self.y,self.yaw= state     
    
    def get_current_vertices(self):
        pos      = np.array([self.x,self.y])
        vertices = pos+EnvMap.rot2d(self.vertices,self.yaw)
        
        return vertices        
    #  here the equations of motion are defined
    
    def get_current_position(self):
        return np.array([self.x,self.y])
    
    
    def step(self,vx,phi) :
        
        # ----------DESCRIPTION--------------
        # define one step for the dynamical
        # model of the robot
        # -----------INPUT-------------------
        # NONE
        # -----------OUTPUT------------------
        # updates state of the system

        # Inspired by https://github.com/winstxnhdw/KinematicBicycleModel/blob/main/kinematic_model.py

        x_dot    = vx * np.cos(self.yaw)
        y_dot    = vx * np.sin(self.yaw)
        yaw_rate = vx * np.tan(self.phi) / self.L

        # Compute the final state
        self.x += x_dot * self.dt
        self.y += y_dot * self.dt
        self.yaw += yaw_rate * self.dt

    def change_angle(self, angle):
        max_angle = self.max_phi

        # Keep angle within allowed angles
        if angle > max_angle:
            angle = max_angle
        elif angle < -max_angle:
            angle = -max_angle

        # Update the angle
        self.phi = angle
        # Update the state
        self.state = np.array([self.x,self.y,self.vx,self.phi])
        
    # draw the robot on the screen
    def draw_robot(self,screen):
        # ----------DESCRIPTION--------------
        # draw robot state on the screen
        # -----------INPUT-------------------
        # screen             ----> pygame.screen object
        # -----------OUTPUT------------------
        # updates state of the system
        
        color    = (10,230,100)
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
        self.global_path     = [[]] # strores all the paths from parent to child node
        self.current_best_cost = None # best cost to path
        
        # obstacles
        self.obstacles = obsList
        # path
        self.goalstate = None
        self.path      = []
        
        
        # Now the robot class is given to the 
        # path planner so that it can use 
        # the robot definition to plan the path
         
        self.robot = robot # robot instanciation
        
    
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
        self.global_path.insert(n,self.local_best_path[:,:2]) # save only the position and not the angle

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
        pathfinder         = Dubin(self.nodes[n1],self.nodes[n2][:2])
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
        rebase_rad = self.maxstep*20
        promising_nodes = []
        for i in range(0, n):
            dist = self.euclide_distance(i, n)
            if dist <rebase_rad :
                promising_nodes.append(i) 
        
        nnear = promising_nodes[0]
        best_dubin_dist,best_path = self.dubin_distance(promising_nodes.pop(0), n)
        self.local_best_path = best_path
        for good_node in promising_nodes:
            
            new_dubin_dist,new_path = self.dubin_distance(good_node, n)
            if best_dubin_dist>new_dubin_dist :
                
                best_dubin_dist      = new_dubin_dist
                self.local_best_path = new_path
                nnear                = good_node
                
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

    def crossObstacle(self,n1,n2):
        #n1 start node
        #n2 end   node
        
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
        
        iscrossing,final_state=self.crossObstacle(n1,n2)
        
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
            self.addCost(n2,best_dubin_dist)
            
            if self.goalFlag and not self.alreadyFound :
                self.goalstate = n2
                self.current_best_cost = self.cost[self.goalstate]
                self.goalFlag  = False
                self.alreadyFound = True
            elif self.goalFlag and self.alreadyFound :
                if self.current_best_cost > self.cost[n2] :
                    self.goalstate = n2
                    self.current_best_cost = self.cost[self.goalstate]
                    self.goalFlag  = False
                    

                    
            
            return True
 
    def step(self, nnear, nrand):
        # takes a step in a random dierection
        # if the new node is too distant from
        # the nearest node, a smaller step
        # is taken with dubin_distance dmax
        
        d = self.euclide_distance(nnear, nrand)
        # definition of distance max is crucial
        
        if d > self.maxstep:
            # take a straight line in the direction
            # of the far point which is as big
            # as the maximum allowed distance
            
            xnear, ynear = self.nodes[nnear][0], self.nodes[nnear][1]
            xrand, yrand = self.nodes[nrand][0], self.nodes[nrand][1]
            
            (px, py)     = (xrand - xnear, yrand - ynear)
            theta        = np.arctan2(py, px)
            (x, y)       = (int(xnear + self.maxstep*np.cos(theta)), int(ynear + self.maxstep*np.sin(theta)))
            
            self.removeNode(nrand)
            
            #                                   # by the dubin path later
            
            if abs(x - self.goal[0]) < self.maxstep and abs(y - self.goal[1]) < self.maxstep:
                # if the goal is in the line add the goal 
                # as a final node ad flag it 
                self.addNode(nrand, (self.goal[0], self.goal[1], theta))
                self.goalFlag=True
            else:
                self.addNode(nrand, (x, y,0)) # the actual theta is calculated 
                                              # by the dubin path later
    
    def isGoalReached(self):
        return self.goalFlag
    
    def isOnePathFound(self):
        return self.alreadyFound

    def getFinalPath(self):
        if self.alreadyFound:
            self.best_final_path=np.flipud(self.global_path[self.goalstate]).tolist()
            totalcost = self.cost[self.goalstate]
            parent    = self.parent[self.goalstate]
            while (parent != 0):
                self.best_final_path+=(np.flipud(self.global_path[parent]).tolist())
                parent = self.parent[parent]
        return self.best_final_path,totalcost
    

       
    def bias(self, ngoal):
        """Take step directly towards the goal"""
        n = self.numberOfNodes()
        self.addNode(n, (ngoal[0], ngoal[1], ngoal[2]))
        nnear = self.euclidean_nearest(n)
        self.step(nnear, n)
        xnearest,best_dubin_dist = self.dubin_nearest(n)
        self.connect(xnearest, n)
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

    