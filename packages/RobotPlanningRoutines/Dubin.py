import numpy as np
from matplotlib import pyplot as plt

# q = [x, y, theta] with theta measured from global x axis (vertical) and CCW positive
# R = radius
# res = samples

class Dubin:
    def __init__(self, q0, q1, R=1., res=None):
        self.radius = R
        self.pos0 = np.array(q0[:2])
        self.pos1 = np.array(q1[:2])
        self.theta0 = q0[2]
        self.theta1 = q1[2]

        self.circles0 = get_circles(self.pos0, self.theta0, self.radius)
        self.circles1 = get_circles(self.pos1, self.theta1, self.radius)

        self.PATHS = np.array([], dtype=tuple)

    def get_path(pos1, circle1, pos2, circle2, R=1.):
        c1 = circle1[0]
        c2 = circle2[0]

        # vectors from centers of the circles to position of the states
        rp1 = pos1 - c1
        rp2 = pos2 - c2

        # vectors from centers of the circles to their respective tangents
        t1, t2 = get_tangents(circle1, circle2, R)
        rt1 = t1 - c1
        rt2 = t2 - c2

        # compute arclengths
        a1 = np.arctan2(rt1[1], rt1[0]) - np.arctan2(rp1[1],
                                                     rp1[0])  # angle from pos to tangent around the circle1 center
        bool = (np.sign(a1) != circle1[-1])  # check if direction of angle and circle are not identical, eg not both CCW
        a1 = abs(2 * np.pi * bool - abs(a1))

        a2 = np.arctan2(rp2[1], rp2[0]) - np.arctan2(rt2[1],
                                                     rt2[0])  # angle from tangent to pos around the circle2 center
        bool = (np.sign(a2) != circle2[-1])  # check if direction of angle and circle are not identical, eg not both CCW
        a2 = abs(2 * np.pi * bool - abs(a2))

        return (pos1, a1, t1, t2, a2, pos2, R)

    def paths(self):
        for circle0 in self.circles0:
            for circle1 in self.circles1:
                path = get_path(self.pos0, circle0, self.pos1, circle1, self.radius)
                length = get_length(path)

                self.PATHS = np.append(self.PATHS, path)

    def plot(self):
        plot_state(q0, self.circles0, self.radius, 'g')
        plot_state(q1, self.circles1, self.radius, 'r')
        plt.gca().set_aspect(1)
        plt.show()


def plot_circle(pos, R=1., color='r'):
    angles = np.arange(0, 2 * np.pi + 0.1, 0.1)
    plt.plot(R * np.cos(angles) + pos[0], R * np.sin(angles) + pos[1], '-.', color=color)

def plot_state(q, circles, R=1., color='r'):
    # Plot the circles
    plot_circle(circles[0][0], R, color)
    plot_circle(circles[1][0], R, color)

    plt.plot(q[0], q[1], 'o', color=color) # plot robot center
    plt.quiver(q[0], q[1], np.cos(q[2]), np.sin(q[2]), scale=5) # plot arrow in the direction of orientation
    plt.plot(circles[0][0][0], circles[0][0][1], '+', color=color)
    plt.plot(circles[1][0][0], circles[1][0][1], '_', color=color)

def rot2d(vec, angle):
    mat = np.array([[np.cos(angle), -np.sin(angle)],
                    [np.sin(angle), np.cos(angle)]])
    new = mat @ vec
    return new

def get_circles(pos, theta, R = 1.):
    cl = pos + R * np.array([np.cos(theta + np.pi/2), np.sin(theta + np.pi/2)])
    cr = pos + R * np.array([np.cos(theta - np.pi / 2), np.sin(theta - np.pi / 2)])

    return (cl, 1), (cr, -1)

def get_tangents(circle1, circle2, R=1.):
    # Note:
    # since the two circles that we are calculating the common tangents for are of the same radius, lots of simplifications can be made.
    # this code would not at all work for the general case of two different sized circles.

    p1 = circle1[0]
    p2 = circle2[0]

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



def get_length(path):
    arc1 = path[0] * path[-1]
    arc2 = path[-2] * path[-1]
    L = np.linalg.norm(path[2]-path[1])
    length = arc1 + arc2 + L
    return arc1, L, arc2, length

def sample_points(path, length, n_points):
    a1_samples = n_samples * (length[0] / length[-1])
    a2_samples = n_samples * (length[2] / length[-1])
    L_samples = n_samples - a1_samples - a2_samples

    points = np.zeros(n_samples, 2)




#################
# TESTS
#################

np.random.seed(100)

q0 = [np.random.randint(-5,5), np.random.randint(-5,5), np.random.random_sample()*2*np.pi]
q1 = [np.random.randint(-5,5), np.random.randint(-5,5), np.random.random_sample()*2*np.pi]

radius = 1.5

d = Dubin(q0, q1, radius)
d.sample()



