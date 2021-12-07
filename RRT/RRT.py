import pygame
from RRTbase import RRTGraph
from RRTbase import RRTMap
import time
from scipy import interpolate
import numpy as np

def main():
    dimensions = (1200, 2000)
    start = (50, 50)
    goal = (1950, 1150)
    obsDim = 50
    obsNum = 200
    i = 0
    t1 = 0
    coords = []
    smooth = []

    pygame.init()
    map = RRTMap(start, goal, dimensions, obsDim, obsNum)
    graph = RRTGraph(start, goal, dimensions, obsDim, obsNum)

    obstacles = graph.makeObs()
    map.drawMap(obstacles)

    t1 = time.time()
    while (not graph.pathToGoal()):
        elapsed = time.time() - t1
        t1 = time.time()
        if elapsed > 10:
            raise

        if i % 10 == 0:
            x, y, parent = graph.bias(goal)
            pygame.draw.circle(map.map, map.grey, (x[-1], y[-1]), map.nodeRad+2, 0)
            pygame.draw.line(map.map, map.blue, (x[-1], y[-1]), (x[parent[-1]], y[parent[-1]]), map.edgeThickness)
        else:
            x, y, parent = graph.expand()
            pygame.draw.circle(map.map, map.grey, (x[-1], y[-1]), map.nodeRad+2, 0)
            pygame.draw.line(map.map, map.blue, (x[-1], y[-1]), (x[parent[-1]], y[parent[-1]]), map.edgeThickness)
        if i % 10 == 0:
            pygame.display.update()

        #smooth.append(graph.getPathCoords())

        i += 1

    map.drawPath(graph.getPathCoords())

    pygame.display.update()
    pygame.event.clear()
    pygame.event.wait(0)

if __name__ == "__main__":
    result = False
    while not result:
        try:
            main()
            result = True
        except:
            result = False
