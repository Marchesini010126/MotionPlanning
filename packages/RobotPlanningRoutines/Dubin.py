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

    return (x_c1, y_c1, 1), (x_c2, y_c2, -1), R

def plot_state(q, R=1., color='r'):
    c1, c2, R = get_circles(q[0], q[1], q[2], R) # c1 is the L-handed circle '1', c2 the R-handed '-1'

    # Plot the circles
    plot_circle(*c1[:-1], R, color)
    plot_circle(*c2[:-1], R, color)

    plt.plot(q[0], q[1], 'x', color=color) # plot robot center
    plt.quiver(q[0], q[1], np.cos(q[2]), np.sin(q[2]), scale=5) # plot arrow in the direction of orientation
    plt.plot(*c1[:-1], '+', color=color)
    plt.plot(*c2[:-1], '_', color=color)

def get_tangents(circle1, circle2, R):
    # outer tangents
    if circle1[-1] == circle2[-1]:

        p1 = np.array([*circle1[:-1]])
        p2 = np.array([*circle2[:-1]])

        V = p2 - p1

        n = np.array([V[1], -V[0]])
        n /= np.linalg.norm(n)
        n *= circle1[-1]

        t1 = p1 + R * n
        t2 = p2 + R * n

        plt.plot([t1[0], t2[0]], [t1[1], t2[1]], '--')

    # inner tangents
    else:
        pass

#################
# TESTS
#################

np.random.seed(8)

q0 = [np.random.randint(-5,5), np.random.randint(-5,5), np.random.rand(1)*2*np.pi]
q1 = [np.random.randint(-5,5), np.random.randint(-5,5), np.random.rand(1)*2*np.pi]

radius = 1.

plot_state(q0, R=radius, color='g')
plot_state(q1, R=radius, color='r')

c0, c1, R = get_circles(q0[0], q0[1], q0[2], R=radius)
c2, c3, _    = get_circles(q1[0], q1[1], q1[2], R=radius)

for circle1 in [c0, c1]:
    for circle2 in [c2, c3]:
        get_tangents(circle1,circle2,R)

plt.gca().set_aspect(1)
plt.show()
