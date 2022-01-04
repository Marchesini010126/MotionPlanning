import math
import random
import pygame
from scipy import interpolate

class RRTMap:
    def __init__(self, start, goal, MapDimensions, obsdim, obsnum):
        self.start = start
        self.goal = goal
        self.MapDimensions = MapDimensions
        self.Maph, self.Mapw = self.MapDimensions

        # Window settings
        self.MapWindowName = "RRT Path Planning"
        pygame.display.set_caption(self.MapWindowName)
        self.map = pygame.display.set_mode((self.Mapw, self.Maph))
        self.map.fill((255, 255, 255))
        self.nodeRad = 2
        self.nodeThickness = 0
        self.edgeThickness = 1

        self.obstacles = []
        self.obsDim = obsdim
        self.obsNum = obsnum

        # Colours
        self.grey = (70, 70, 70)
        self.blue = (0, 0, 255)
        self.green = (0, 255, 0)
        self.red = (255, 0, 0)
        self.white = (255, 255, 255)



    def drawMap(self, obstacles):
        pygame.draw.circle(self.map, self.green, self.start[:2], self.nodeRad+5, 0)
        pygame.draw.circle(self.map, self.green, self.goal[:2], self.nodeRad+20, 1)
        self.drawObs(obstacles)

    def drawPath(self, path):
        for node in path:
            pygame.draw.circle(self.map, self.red, node[:2], self.nodeRad+3, 0)

    def drawObs(self, obstacles):
        obstaclesList = obstacles.copy()
        while(len(obstaclesList) > 0):
            obstacle = obstaclesList.pop(0)
            pygame.draw.rect(self.map, self.grey, obstacle)




class RRTGraph:
    def __init__(self, start, goal, MapDimensions, obsdim, obsnum):
        self.start = start
        self.goal = goal
        self.goalFlag = False
        self.maph, self.mapw = MapDimensions
        # initialize tree
        self.nodes = [start]  # List of tuples representing C space (x, y, theta)
        self.parent = [0]  # List of ints, representing each node's parent (ie node N's parent can be found at index N)
        # obstacles
        self.obstacles = []
        self.obsDim = obsdim
        self.obsNum = obsnum
        # path
        self.goalstate = None
        self.path = []


    def makeRandomRect(self):
        """Randomly select coordinates in workspace"""
        uppercornerx = int(random.uniform(0, self.mapw - self.obsDim))
        uppercornery = int(random.uniform(0, self.maph - self.obsDim))
        return (uppercornerx, uppercornery)


    def makeObs(self):
        """Create given amount of obstacles, clear of start and goal point"""
        obs = []
        for i in range(0, self.obsNum):
            rectangle = None
            startgoalcol = True
            while startgoalcol:
                upper = self.makeRandomRect()
                rectangle = pygame.Rect(upper, (self.obsDim, self.obsDim))
                if rectangle.collidepoint(self.start[:2]) or rectangle.collidepoint(self.goal[:2]):
                    startgoalcol = True
                else:
                    startgoalcol = False
            obs.append(rectangle)
        self.obstacles = obs.copy()
        return obs

    def addNode(self, n, node):
        """Add node to graph

        Keyword arguments:
        n -- the node number
        node -- tuple represent C-space: (x, y, theta)
        """
        self.nodes.insert(n, node)


    def removeNode(self, n):
        """Remove node from graph"""
        self.nodes.pop(n)

    def addEdge(self, parent, child):
        """Connect nodes to each other"""
        self.parent.insert(child, parent)

    def removeEdge(self, n):
        """Remove connection between nodes"""
        self.parent.pop(n)

    def numberOfNodes(self):
        """Return number of nodes in graph"""
        return len(self.nodes)

    def distance(self, n1, n2):
        """Calculate straight line distance between two nodes"""
        node1, node2 = self.nodes[n1], self.nodes[n2]
        x1, y1, theta1 = node1[0], node1[1], node1[2]
        x2, y2, theta2 = node2[0], node2[1], node2[2]
        px = (float(x1) - float(x2))**2
        py = (float(y1) - float(y2))**2
        return (px + py)**(0.5)

    def sample_envir(self):
        """Get random coordinate in workspace"""
        x = int(random.uniform(0, self.mapw))
        y = int(random.uniform(0, self.maph))
        return x, y

    def nearest(self, n):
        """Return nearest connected node"""
        dmin = self.distance(0, n)
        nnear = 0
        for i in range(0, n):
            if self.distance(i, n) < dmin:
                dmin = self.distance(i, n)
                nnear = i
        return nnear

    def isFree(self):
        """Check if newest node is not within an object"""
        n = self.numberOfNodes() - 1
        (x, y, theta) = self.nodes[n]
        obs = self.obstacles.copy()
        while len(obs) > 0:
            rectangle = obs.pop(0)
            if rectangle.collidepoint(x, y):
                self.removeNode(n)
                return False
        return True

    def crossObstacle(self, x1, x2, y1, y2):
        """Check if straight line between two coordinates crosses objects"""
        obs = self.obstacles.copy()
        while len(obs) > 0:
            rectangle = obs.pop(0)
            for i in range(0, 101):
                u = i/100
                x = x1*u + x2*(1 - u)
                y = y1 * u + y2 * (1 - u)
                if rectangle.collidepoint(x, y):
                    return True
        return  False

    def connect(self, n1, n2):
        """Connect two nodes, if possible"""
        node1, node2 = self.nodes[n1], self.nodes[n2]
        x1, y1, theta1 = node1[0], node1[1], node1[2]
        x2, y2, theta2 = node2[0], node2[1], node2[2]
        if self.crossObstacle(x1, x2, y1, y2):
            self.removeNode(n2)
            return False
        else:
            self.addEdge(n1, n2)
            return True

    def step(self, nnear, nrand, dmax = 35):
        """Check if new node is close enough to nearest node, if not move it closer."""
        d = self.distance(nnear, nrand)
        if d > dmax:
            u = dmax/d
            xnear, ynear = self.nodes[nnear][0], self.nodes[nnear][1]
            xrand, yrand = self.nodes[nrand][0], self.nodes[nrand][1]
            (px, py) = (xrand - xnear, yrand - ynear)
            theta = math.atan2(py, px)
            (x, y) = (int(xnear + dmax*math.cos(theta)), int(ynear + dmax*math.sin(theta)))
            self.removeNode(nrand)
            if abs(x - self.goal[0]) < dmax and abs(y - self.goal[1]) < dmax:
                self.addNode(nrand, (self.goal[0], self.goal[1], theta))
                self.goalstate = nrand
                self.goalFlag = True
            else:
                self.addNode(nrand, (x, y, theta))


    def pathToGoal(self):
        """Trace back steps to find th path after goal is reached"""
        if self.goalFlag:
            self.path = []
            self.path.append(self.goalstate)
            newpos = self.parent[self.goalstate]
            while newpos != 0:
                self.path.append(newpos)
                newpos = self.parent[newpos]
            self.path.append(0)
        return self.goalFlag

    def getPathCoords(self):
        """Return list of each coordinate (in C space) of the path"""
        pathCoords = []
        for node in self.path:
            pathCoords.append(self.nodes[node])
        return pathCoords

    def bias(self, ngoal):
        """Take step directly towards the goal"""
        n = self.numberOfNodes()
        self.addNode(n, (ngoal[0], ngoal[1], ngoal[2]))
        nnear = self.nearest(n)
        self.step(nnear, n)
        self.connect(nnear, n)
        return self.nodes, self.parent

    def expand(self):
        """Take random step"""
        n = self.numberOfNodes()
        x, y = self.sample_envir()
        self.addNode(n, (x, y, 0))  # !! theta randomly set to 0, as it is currently ignored. To be calculated by Dubin
        if self.isFree():
            xnearest = self.nearest(n)
            self.step(xnearest, n)
            self.connect(xnearest, n)
        return self.nodes, self.parent

    def cost(self):
        pass

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
