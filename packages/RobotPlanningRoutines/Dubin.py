import numpy as np
from matplotlib import pyplot as plt

# q = [x, y, theta] with theta measured from global x axis (vertical) and CCW positive
# R = radius
# res = samples

class Dubin:
    def __init__(self, q0, q1, R=1., res=None):
        self.radius = R


def plot_circle(x, y, R=1., color='r'):
    angles = np.arange(0, 2 * np.pi + 0.1, 0.1)
    plt.plot(R * np.cos(angles) + x, R * np.sin(angles) + y, '-.', color=color)


def plot_state(q, R=1., color='r'):
    c1, c2, R = get_circles(q[0], q[1], q[2], R) # c1 is the L-handed circle '1', c2 the R-handed '-1'

    # Plot the circles
    plot_circle(*c1[:-1], R, color)
    plot_circle(*c2[:-1], R, color)

    plt.plot(q[0], q[1], 'x', color=color) # plot robot center
    plt.quiver(q[0], q[1], np.cos(q[2]), np.sin(q[2]), scale=5) # plot arrow in the direction of orientation
    plt.plot(*c1[:-1], '+', color=color)
    plt.plot(*c2[:-1], '_', color=color)


def rot2d(vec, angle):
    mat = np.array([[np.cos(angle), -np.sin(angle)],
                    [np.sin(angle), np.cos(angle)]])
    new = mat @ vec
    return new


def get_circles(x, y, theta, R = 1.):
    x_c1 = x + R * np.cos(theta + np.pi/2)
    y_c1 = y + R * np.sin(theta + np.pi/2)

    x_c2 = x + R * np.cos(theta - np.pi / 2)
    y_c2 = y + R * np.sin(theta - np.pi / 2)

    return (x_c1, y_c1, 1), (x_c2, y_c2, -1), R


def get_tangents(circle1, circle2, R=1.):
    # Note:
    # since the two circles that we are calculating the common tangents for are of the same radius, lots of simplifications can be made.
    # this code would not at all work for the general case of two different sized circles.

    p1 = np.array([*circle1[:-1]])
    p2 = np.array([*circle2[:-1]])

    V = p2 - p1 # vector V from center of cirlce 1 to cirlce 2

    # outer tangents
    if circle1[-1] == circle2[-1]:
        n = np.array([V[1], -V[0]]) # normal vector to V: if V has slope v/u, then the normal has slope -u/v
        n /= np.linalg.norm(n)
        n *= circle1[-1] # the placement of the minus sign in "-u/v" matters for distinguishing outer tangents of CW and CCW circles: [v, -u] is not [-v, u]

        t1 = p1 + R * n
        t2 = p2 + R * n

    # inner tangents
    else:
        D = np.linalg.norm(V)
        alpha = np.arccos(R/(0.5*D)) # we know the inner tangents will intersect in the middle
        alpha *= circle2[-1]

        n = rot2d(V, alpha) # vector pointing from circle center to tangent point
        n /= np.linalg.norm(n)

        t1 = p1 + n * R
        t2 = p2 - n * R

    return t1, t2

def get_path(p1, circle1, p2, circle2, R=1.):
    c1 = np.array([*circle1[:-1]])
    c2 = np.array([*circle2[:-1]])

    r1 = np.array([p1]).T - c1
    r2 = np.array([p2]).T - c2

    t1, t2 = get_tangents(circle1, circle2, R)

    t = t2 - t1
    t1 -= c1
    t2 -= c2

    a1 = np.arctan2(t1[1], t1[0]) - np.arctan2(r1[1], r1[0]) # angle from pos to tangent around the circle1 center
    bool = (np.sign(a1) != circle1[-1]) # check if direction of angle and circle are not identical, eg not both CCW
    a1 = abs(2 * np.pi * bool - abs(a1))
    arc1 = R * a1

    a2 = np.arctan2(r2[1], r2[0]) - np.arctan2(t2[1], t2[0])  # angle from tangent to pos around the circle2 center
    bool = (np.sign(a2) != circle2[-1])  # check if direction of angle and circle are not identical, eg not both CCW
    a2 = abs(2 * np.pi * bool - abs(a2))
    arc2 = R * a2

    L = np.linalg.norm(t)

    return arc1, L, arc2


#################
# TESTS
#################

np.random.seed(100)

q0 = [np.random.randint(-5,5), np.random.randint(-5,5), np.random.rand(1)*2*np.pi]
q1 = [np.random.randint(-5,5), np.random.randint(-5,5), np.random.rand(1)*2*np.pi]

radius = 1.5

plot_state(q0, R=radius, color='g')
plot_state(q1, R=radius, color='r')

c0, c1, _ = get_circles(q0[0], q0[1], q0[2], R=radius)
c2, c3, _ = get_circles(q1[0], q1[1], q1[2], R=radius)

for circle1 in [c0, c1]:
    for circle2 in [c2, c3]:
        t1, t2 = get_tangents(circle1,circle2, radius)
        arc1, L, arc2 = get_path(q0[:2], circle1, q1[:2], circle2, radius)
        print(arc1, L, arc2)
        plt.plot([t1[0], t2[0]], [t1[1], t2[1]], '--y')

plt.gca().set_aspect(1)
plt.show()
