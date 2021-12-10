import matplotlib.patches
import pygame as pg
import numpy as np
from matplotlib import pyplot as plt

# q = [x, y, theta] with theta measured from global x axis (vertical) and CCW positive
# R = radius
# res = samples

def get_circles(x, y, theta, R = 5.):
    x_c1 = R * np.cos(theta + np.pi/2)
    y_c1 = R * np.sin(theta + np.pi/2)

    x_c2 = R * np.cos(theta - np.pi / 2)
    y_c2 = R * np.sin(theta - np.pi / 2)

    return (x_c1, y_c1), (x_c2, y_c2)


# TESTS
x = 5
y = 3
theta = np.pi/6

c1, c2 = get_circles(x, y, theta)

circle1 = plt.Circle(c1, radius=5.)
plt.add_artist(circle1)
plt.show()

print(circle1)

