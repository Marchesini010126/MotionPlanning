import matplotlib.patches
import pygame as pg
import numpy as np
from matplotlib import pyplot as plt

# q = [x, y, theta] with theta measured from global x axis (vertical) and CCW positive
# R = radius
# res = samples

def get_circles(x, y, theta, R = 5.):
    x_c1 = x + R * np.cos(theta + np.pi/2)
    y_c1 = y + R * np.sin(theta + np.pi/2)

    x_c2 = x + R * np.cos(theta - np.pi / 2)
    y_c2 = y + R * np.sin(theta - np.pi / 2)

    # Plot the circles
    angles = np.arange(0,2*np.pi+0.1, 0.1)
    plt.plot(R * np.cos(angles) + x_c1, R * np.sin(angles) + y_c1,'-.', 'b')
    plt.plot(R * np.cos(angles) + x_c2, R * np.sin(angles) + y_c2, '-.', 'r')
    plt.plot(x, y, 'x')
    plt.plot(x_c1, y_c1, '.', 'b')
    plt.plot(x_c2, y_c2, '.', 'r')

    plt.show()

    return (x_c1, y_c1), (x_c2, y_c2), R


# TESTS
x = 5
y = 3
theta = np.pi/6

c1, c2 = get_circles(x, y, theta)


