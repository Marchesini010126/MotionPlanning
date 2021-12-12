import math
import random
import pygame
import sys
import numpy as np

from   scipy.spatial import distance
from   scipy import interpolate
from   random import randint


if __name__ == "__main__" :
    import ObstaclesFactory
    import CollisionChecks
else :
    from  RobotPlanningRoutines import ObstaclesFactory
    from  RobotPlanningRoutines import CollisionChecks
    

# there is a problem here 
# if you run the code directly 
# the robotPlanningRoutines will not load


class EnvMap():
    
    def __init__(self,start, goal, MapDimensions) :
        
        # state initialization
        self.start = start    # initial state --> tuple
        self.goal  = goal     # goal state    --> tuple
        
        # screen dimension
        self.MapDimensions   = MapDimensions
        self.Mapw,self.Maph = self.MapDimensions
        
        self.obstacle_list   = None  #list of obstacles
        
        # Window settings
        self.MapWindowName = "RRT Path Planning"
        pygame.display.set_caption(self.MapWindowName)
        self.map = pygame.display.set_mode((self.Mapw, self.Maph)) # creates the screen
        self.map.fill((0, 0, 0))                             # bkg color
        
        
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
    

    # getters and setters
    def get_obstacles(self):
        return self.obstacle_list
        
        
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
     
    def draw_startgoal(self):
        pygame.draw.circle(self.map,self.green,self.start,4)
        pygame.draw.circle(self.map,self.green,self.goal,4)
        
        
    def draw_obstacles(self) :
        
        # ----------DESCRIPTION--------------
        # draw obstacles in the screen
        # -----------INPUT-------------------
        # screen             ----> pygame.screen object
        # -----------OUTPUT------------------

        
        for obstacle in self.obstacle_list :
            if obstacle['type'] == 'circle' :
                pygame.draw.circle(self.map,obstacle['color'],tuple(obstacle['center']),obstacle['radius'])
            else :
                pygame.draw.polygon(self.map,obstacle['color'],tuple(obstacle['vertices']))
    
    
    def drawPath(self, path):
        # path : list of tuple (x,y)
        for node in path:
            pygame.draw.circle(self.map, self.blue, node, self.nodeRad+3, 0)
     
    
    
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
            rad      = random.randrange(20,maxRadius)
            center   = (random.randint(0,self.Mapw),random.randint(0,self.Maph))
            
            if obstype == 'polygon' :
                nfaces   = random.randint(3,maxFaces) 
                obstacle = ObstaclesFactory.createNpolygon(nfaces,rad,center)
                
            elif obstype == 'rectangle' :
                width    = random.randint(0,100) 
                height   = random.randint(0,100)
                obstacle = ObstaclesFactory.createRectangle(width,height,center)
            
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
    # for now the robot has only circluar shape
    # it will ba updated as soon as possible 
    
    def __init__(self,start_loc=(0,0),radius=20) :
        
        # state of the system
        self.start_loc = start_loc
        self.vx0       = 0.0   # m/s
        self.vy0       = 0.0   # m/s
        self.stert_speed = (self.vx0,self.vy0)
        
        # define the state
        self.state   = np.array([self.start_loc[0],self.start_loc[1],self.vx0,self.vy0])
        
        # dynamic parameters   
        self.dt      = 0.01 # s         # integration step
        self.mass    = 10   # kg        # mass
        self.gravity = 9.81 # pixel/s2      # gravity acceleration
        
        self.radius  = radius
        # add capability to change it 
        # available_shapes = {'rectangle','circle','polygon'}
        # factory          = ObstaclesFactory()  
        # if self.type == 'rectangle' :
        #     pass
                      
    #  here the equaltions of motion are defined
    def motion(self,u) :
        
        # ----------DESCRIPTION--------------
        # define one step for the dynamical
        # model of the robot
        # -----------INPUT-------------------
        # u    control action  ---> np.array()
        # -----------OUTPUT------------------
        # updates state of the system
        
        
       #TODO 
        pass
        
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
            
            
## RRT Class

class RRTGraph:
    def __init__(self, start, goal, MapDimensions,obsList,RobotClass):
        (x, y) = start
        self.start = start
        self.goal = goal
        self.goalFlag = False
        self.mapw, self.maph, = MapDimensions
        self.x = []
        self.y = []
        self.parent = []
        self.cost   =[]
        
        # initialize tree
        self.x.append(x)
        self.y.append(y)
        self.parent.append(0)
        self.cost.append(0)
        
        
        # obstacles
        self.obstacles = obsList
        # path
        self.goalstate = None
        self.path = []
        
        # Now the robot class is given to the 
        # path planner so that it can use 
        # the robot definition to plan the path
         
        self.robot = RobotClass # np.array as for obstacles 
        
    
    def check_clear_startgoal(self):
        # eliminated the obstacles that are 
        # exactly on the goal or the start position
        
        # retruns a list to update the EnvMap
        # class
        # from tuple to array
        start = np.asarray(self.start,dtype=np.float32)
        goal  = np.asarray(self.goal,dtype=np.float32)
        
        for n,obstacle in enumerate(self.obstacles) :
            
            check1    = CollisionChecks.CircleCollision(obstacle['radious'],self.robot.radius,obstacle['center'],start)
            check2    = CollisionChecks.CircleCollision(obstacle['radious'],self.robot.radius,obstacle['center'],goal) 
            if check1 or check2 :
                self.obstacles.pop(n)
        
        return self.obstacles
        
        
       

    def addNode(self, n, x, y):
        self.x.insert(n, x)
        self.y.insert(n, y)


    def removeNode(self, n):
        self.x.pop(n)
        self.y.pop(n)

    def addEdge(self, parent, child):
        self.parent.insert(child, parent)

    def removeEdge(self, n):
        self.parent.pop(n)
    
    def addCost(self, n,distance) :
        self.cost.insert(n,self.cost[self.parent[n]]+distance)
    
    def removeCost(self, n) :
        self.cost.pop(n)
    
    def getCost(self, n) :
        return self.cost[n]
        
    
    def numberOfNodes(self):
        return len(self.x)

    def distance(self, n1, n2):
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])
        px = (float(x1) - float(x2))**2
        py = (float(y1) - float(y2))**2
        return (px + py)**(0.5)

    def sample_envir(self):
        x = int(random.uniform(0, self.mapw))
        y = int(random.uniform(0, self.maph))
        return x, y

    def nearest(self, n):
        dmin = self.distance(0, n)
        nnear = 0
        for i in range(0, n):
            if self.distance(i, n) < dmin:
                dmin = self.distance(i, n)
                nnear = i
        return nnear

    def isFree(self):
        n = self.numberOfNodes() - 1
        (x, y) = self.x[n], self.y[n]
        obs    = self.obstacles.copy()
        
        while len(obs) > 0:
            obstacle = obs.pop(0)
            obs_vert   = obstacle['vertices']
            # we will need to add rotations as well
            robot_pos = np.array([x,y])
            # hirerchical checking
            if CollisionChecks.CircleCollision(obstacle['radius'],self.robot.radius,obstacle['center'],robot_pos) :
                if CollisionChecks.GJK(robot_pos,obs_vert,radius=self.robot.radius): # the robot is a circle for now. So GJK is polygon vs Circle
                   self.removeNode(n)
                   return False
        return True

    def crossObstacle(self, x1, x2, y1, y2):
        obs = self.obstacles.copy()
        while len(obs) > 0:
            obstacle = obs.pop(0)
            obs_vert   = obstacle['vertices']
            for i in range(0, 101):
                u = i/100
                x = x1*u + x2*(1 - u)
                y = y1 * u + y2 * (1 - u)
                robot_pos = np.array([x,y])
                # hirerchical checking
                if x > 0 or x < self.mapw or y >0 or y <self.maph :
                    if CollisionChecks.CircleCollision(obstacle['radius'],self.robot.radius,obstacle['center'],robot_pos) :
                        if CollisionChecks.GJK(robot_pos,obs_vert,radius=self.robot.radius): # the robot is a circle for now. So GJK is polygon vs Circle
                            return True
        return False

    def connect(self, n1, n2):
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])
        if self.crossObstacle(x1, x2, y1, y2):
            self.removeNode(n2)
            return False
        else:
            self.addEdge(n1, n2)
            return True
    
    def rebase_connection(self,n1,n2) :
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])
        if self.crossObstacle(x1, x2, y1, y2):
            return False
        else:
            return True
        
        
        
    def step(self, nnear, nrand, dmax = 35):
        d = self.distance(nnear, nrand)
        cost = d
        if d > dmax:
            cost = dmax
            u = dmax/d
            (xnear, ynear) = (self.x[nnear], self.y[nnear])
            (xrand, yrand) = (self.x[nrand], self.y[nrand])
            (px, py) = (xrand - xnear, yrand - ynear)
            theta = math.atan2(py, px)
            (x, y) = (int(xnear + dmax*math.cos(theta)), int(ynear + dmax*math.sin(theta)))
            self.removeNode(nrand)
            if abs(x - self.goal[0]) < dmax and abs(y - self.goal[1]) < dmax:
                self.addNode(nrand, self.goal[0], self.goal[1])
                self.goalstate = nrand
                self.goalFlag = True
            else:
                self.addNode(nrand, x, y)
        return cost

    def rebase(self) :
        # optimal radious to be chnaged in case higher dimension is
        # searched for 
        gamma            =  400# by chance. Good papameter not found
        reabse_radius    =  gamma*np.sqrt(np.log(self.numberOfNodes())/self.numberOfNodes())**(1/2)
        last_node        =  self.numberOfNodes()-1
        mincost          =  self.getCost(last_node) # cost of the last added node
        rebase_candidate = []
        for node in range(len(self.cost)-1) : # exclude last node from search
            dist = self.distance(node,last_node)
            if dist < reabse_radius :
                if dist + self.getCost(node) < mincost :
                    rebase_candidate.append(node)
                    mincost          =  dist + self.getCost(node)  
                    
        if len(rebase_candidate) > 0 : 
           
           isfree   = self.rebase_connection(rebase_candidate[-1], last_node)
           if  isfree : # no crossing obstacle, so I can connect
              self.removeEdge(last_node)
              self.removeCost(last_node)
              self.addEdge(rebase_candidate[-1],last_node)
              self.addCost(last_node,mincost)      
        return self.x, self.y, self.parent
    
    def pathToGoal(self):
        if self.goalFlag:
            self.path = []
            self.path.append(self.goalstate)
            newpos = self.parent[self.goalstate]
            while (newpos != 0):
                self.path.append(newpos)
                newpos = self.parent[newpos]
            self.path.append(0)
        return self.goalFlag

    def getPathCoords(self):
        pathCoords = []
        for node in self.path:
            x, y = (self.x[node], self.y[node])
            pathCoords.append((x, y))
        return pathCoords

    def bias(self, ngoal):
        n = self.numberOfNodes()
        self.addNode(n, ngoal[0], ngoal[1])
        nnear = self.nearest(n)
        cost=self.step(nnear, n)
        isfree=self.connect(nnear, n)
        if  isfree : # no crossing obstacle, so I can connect
                self.addCost(n,cost)
        return self.x, self.y, self.parent

    def expand(self):
        n = self.numberOfNodes()
        x, y = self.sample_envir()
        self.addNode(n, x, y)
        if self.isFree():
            xnearest = self.nearest(n)
            cost     = self.step(xnearest, n)
            isfree   = self.connect(xnearest, n)
            if  isfree : # no crossing obstacle, so I can connect
                self.addCost(n,cost)
        return self.x, self.y, self.parent


    def B_spline(self, coordinates):
        x = []
        y = []

        for point in coordinates:
            x.append(point[0])
            y.append(point[1])

        tck, *rest = interpolate.splprep([x, y])
        u = np.linspace(0, 1, num=100)
        smooth = interpolate.splev(u, tck)
        return smooth
