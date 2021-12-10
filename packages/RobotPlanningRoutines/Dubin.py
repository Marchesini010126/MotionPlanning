import matplotlib.patches
import pygame as pg
import numpy as np
from matplotlib import pyplot as plt

# q = [x, y, theta] with theta measured from global x axis (vertical) and CCW positive
# R = radius
# res = samples

def plot_circle(x, y, R, color='r'):
    angles = np.arange(0, 2 * np.pi + 0.1, 0.1)
    plt.plot(R * np.cos(angles) + x, R * np.sin(angles) + y, '-.', color=color)

def get_circles(x, y, theta, R = 1.):
    x_c1 = x + R * np.cos(theta + np.pi/2)
    y_c1 = y + R * np.sin(theta + np.pi/2)

    x_c2 = x + R * np.cos(theta - np.pi / 2)
    y_c2 = y + R * np.sin(theta - np.pi / 2)

    return (x_c1, y_c1, 'L'), (x_c2, y_c2, 'R'), R

def plot_state(q, R=1., color='r'):
    c1, c2, R = get_circles(q[0], q[1], q[2]) # c1 is the L-handed circle, c2 the R-handed

    # Plot the circles
    plot_circle(*c1[:-1], R, color)
    plot_circle(*c2[:-1], R, color)

    plt.plot(q[0], q[1], 'x', color=color) # plot robot center
    plt.quiver(q[0], q[1], np.cos(q[2]), np.sin(q[2]), scale=5) # plot arrow in the direction of orientation
    plt.plot(*c1[:-1], '+', color=color)
    plt.plot(*c2[:-1], '_', color=color)

def get_tangents(circle1, circle2, R):
    # outer tangents
    p1 = np.array([*circle1[:-1]])
    p2 = np.array([*circle2[:-1]])

    V = p2 - p1

    n = np.array([V[1], -V[0]])
    n /= np.linalg.norm(n)

    t1 = p1 + R * n
    t2 = p2 + R * n

    plt.plot([t1[0], t2[0]], [t1[1], t2[1]], '--')

#################
# TESTS
#################

np.random.seed(8)

q0 = [np.random.randint(-5,5), np.random.randint(-5,5), np.random.rand(1)*2*np.pi]
q1 = [np.random.randint(-5,5), np.random.randint(-5,5), np.random.rand(1)*2*np.pi]

plot_state(q0, color='g')
plot_state(q1, color='r')

c0, c1, R = get_circles(q0[0], q0[1], q0[2])
c2, c3, _    = get_circles(q1[0], q1[1], q1[2])

get_tangents(c0,c2,R)

plt.gca().set_aspect(1)
plt.show()
