import pygame
from RRTbase import RRTGraph
from RRTbase import RRTMap
import time
from scipy import interpolate
import numpy as np

def main():
    dimensions = (750, 1200)
    start = (50, 50, 0)
    goal = (1150, 700, 0)
    obsDim = 50
    obsNum = 120
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
            nodes, parent = graph.bias(goal)
            x, y = nodes[-1][0], nodes[-1][1]
            pygame.draw.circle(map.map, map.grey, (x, y), map.nodeRad+2, 0)
            pygame.draw.line(map.map, map.blue, (x, y), (nodes[parent[-1]][0], nodes[parent[-1]][1]), map.edgeThickness)
        else:
            nodes, parent = graph.expand()
            x, y = nodes[-1][0], nodes[-1][1]
            pygame.draw.circle(map.map, map.grey, (x, y), map.nodeRad+2, 0)
            pygame.draw.line(map.map, map.blue, (x, y), (nodes[parent[-1]][0], nodes[parent[-1]][1]), map.edgeThickness)
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
        except Exception as e:
            result = False
            print(e)
