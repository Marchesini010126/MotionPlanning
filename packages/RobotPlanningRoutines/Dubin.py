import numpy as np
from matplotlib import pyplot as plt

class Dubin:

    """
    Dubin creates a Dubin's path between states q0 and q1, keeping to the minimum turning radius R, divided in n_samples
    that can be used for collision checks.

    INPUT:
        q0: [x0, y0, theta0]    initial state, theta measured CCW positive from global x-axis
        q1: [x1, y1, theta1]    final state

    FUNCTIONS:
        get_circles(position, orientation)
            finds the turn left and turn right circles for a given position and orientation
            Output: two cirlces of type:
                tuple: (np.ndarray: center_pos, int: turning_sign)

        get_path(circle0, circle1)
            finds a single path between two given circles adn their signs
            Output:
                path of type:
                    list: [float: angle to pos0, float: angle to tangent0, ndarray: pos of tangent 0, ndarray: pos of tangent 1, float: angle to tangent1, float: angle to pos1]
                length of type:
                    list: [float: angle on circle0, float: distance between tangents on the two cirlces, float: angle on circle 1, float: total distance]

        make_path()
            finds all paths between two states
            Output:
                PATHS of type:
                    ndarray of shape (num_paths, num_samples, 3):
                        number of paths found (4) sampled at n_samples points, containing the configurations [x, y, theta]

        plot()
            plots q0 and q1 states as two vectors (green for inital, red for final) with the turning circles and their signs,
            if make_path() has been called, the paths are plotted as well, showing up as '.' at each sampled point.
    """

    def __init__(self, q0, q1, R=1., n_samples=None):
        self.radius = R
        self.pos0 = np.array(q0[:2])
        self.pos1 = np.array(q1[:2])
        self.theta0 = q0[2]
        self.theta1 = q1[2]

        self.res = n_samples

        self.circles0 = self.get_circles(self.pos0, self.theta0)
        self.circles1 = self.get_circles(self.pos1, self.theta1)

        self.paths = np.array([])

    def get_circles(self, pos, theta):
        cl = pos + self.radius * np.array([np.cos(theta + np.pi / 2), np.sin(theta + np.pi / 2)])
        cr = pos + self.radius * np.array([np.cos(theta - np.pi / 2), np.sin(theta - np.pi / 2)])

        return (cl, 1), (cr, -1)

    def get_path(self, circle0, circle1):
        c0 = circle0[0]
        c1 = circle1[0]

        # vectors from centers of the circles to position of the states
        rp0 = self.pos0 - c0
        rp1 = self.pos1 - c1

        # vectors from centers of the circles to their respective tangents
        t1, t2 = get_tangents(circle0, circle1, self.radius)
        rt1 = t1 - c0
        rt2 = t2 - c1
        L = np.linalg.norm(t2-t1) # distance from t1 to t2

        # compute arc(lengths)
        alpha_p = np.arctan2(rp0[1], rp0[0])
        alpha_t = np.arctan2(rt1[1], rt1[0])
        d_alpha = alpha_t - alpha_p # angle from pos to tangent around the circle1 center
        bool = (np.sign(d_alpha) != circle0[-1])  # check if direction of angle and circle are not identical, eg not both CCW
        d_alpha = circle0[-1] * abs(2 * np.pi * bool - abs(d_alpha)) # make sure the angle is measured along the direction of the turn

        beta_t = np.arctan2(rt2[1], rt2[0])
        beta_p = np.arctan2(rp1[1], rp1[0])
        d_beta = beta_p - beta_t  # angle from tangent to pos around the circle2 center
        bool = (np.sign(d_beta) != circle1[-1])  # check if direction of angle and circle are not identical, eg not both CCW
        d_beta = circle1[-1] * abs(2 * np.pi * bool - abs(d_beta)) # make sure the angle is measured along the direction of the turn

        total = abs(d_alpha * self.radius) + L + abs(d_beta * self.radius)

        return [alpha_p, alpha_t, t1, t2, beta_t, beta_p], [d_alpha, L, d_beta, total]

    def make_path(self): #TODO: add orientations to the returned states
        self.paths = np.zeros((4, self.res, 3))
        i = 0
        for circle0 in self.circles0:
            for circle1 in self.circles1:
                path, length = self.get_path(circle0, circle1)

                if type(path[2]) == np.ndarray: #TODO: resolve the overlapping circles issue, for now just check
                    alpha_samples = abs(self.res * length[0]*self.radius//length[-1]).astype(np.int64)
                    beta_samples = abs(self.res * length[2]*self.radius//length[-1]).astype(np.int64)
                    L_samples = (self.res - alpha_samples - beta_samples).astype(np.int64)

                    n = 0
                    for a in np.linspace(path[0], path[0]+length[0], alpha_samples):
                        p = circle0[0] + self.radius * np.array([np.cos(a), np.sin(a)])
                        th = a + circle0[-1]*np.pi/2
                        self.paths[i][n][:-1] += p
                        self.paths[i][n][-1] += th
                        n += 1

                    V = path[3] - path[2]
                    th = np.arctan2(V[1], V[0])
                    for l in np.linspace(path[2], path[3], L_samples):
                        self.paths[i][n][:-1] = l
                        self.paths[i][n][-1] += th
                        n += 1

                    for b in np.linspace(path[-2], path[-2]+length[-2], beta_samples):
                        p = circle1[0] + self.radius * np.array([np.cos(b), np.sin(b)])
                        th = b + circle1[-1] * np.pi / 2
                        self.paths[i][n][:-1] += p
                        self.paths[i][n][-1] += th
                        n += 1

                i += 1

        return self.paths

    def plot(self):
        plot_state(q0, self.circles0, self.radius, 'g')
        plot_state(q1, self.circles1, self.radius, 'r')

        for path in self.paths:
            plt.plot(path.T[0], path.T[1], '.')


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

        if D/2 >= R:
            alpha = np.arccos(R / (0.5 * D))  # we know the inner tangents will intersect in the middle
            alpha *= circle2[-1]

            n = rot2d(V, alpha) # vector pointing from circle center to tangent point
            n /= np.linalg.norm(n)

            t1 = p1 + n * R
            t2 = p2 - n * R

        else:
            # the two circles overlap, so no inner tangents can be found:
            # TODO : create two new circles, both tangent to the two original circles and find tangent points between them

            print("No inner tangents between overlapping circles!")
            t1, t2 = np.nan, np.nan

    return t1, t2


#################
# TESTS
#################

np.random.seed(9) # a nice seed is 9, seed for overlapping debug eg.40

q0 = [np.random.randint(-5,5), np.random.randint(-5,5), np.random.random_sample()*2*np.pi]
q1 = [np.random.randint(-5,5), np.random.randint(-5,5), np.random.random_sample()*2*np.pi]
radius = 0.5

d = Dubin(q0, q1, radius, 40)

paths = d.make_path()

print(paths)
d.plot()


