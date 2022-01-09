import numpy as np
from matplotlib import pyplot as plt

class Dubin:

    def __init__(self, start_configuration, goal, Rturn=1., robot_obj=None, n_samples=20):
        
        # ----------DESCRIPTION--------------
        # initialise class
        # -----------INPUT-------------------
        # start_configuration : initial state (x,y,theta)   tuple
        # goal                : goal position (x,y)         tuple
        # Rturn               : turning radius              float
        # n_sample            : number of samples           float
        #                       to be used for a single
        #                       path
        # -----------OUTPUT------------------
        # update state 

        if robot_obj:
            self.radius = robot_obj.vmax/robot_obj.maxyaw
            self.v      = robot_obj.vmax
            self.phi    = robot_obj.phi_max
        else:
            self.radius = Rturn
            self.v = 0.
            self.phi = 0.

        self.start_conf = start_configuration 
        self.pos0       = np.array(start_configuration[:2])
        self.pos1       = np.array(goal)
        self.theta0     = start_configuration[2]

        self.res = n_samples

        self.circles = self.get_circles(self.pos0, self.theta0)

        self.paths = np.array([])

    def get_circles(self, pos, theta):
        """gets starting turning circles"""
        cl = pos + self.radius * np.array([np.cos(theta + np.pi / 2), np.sin(theta + np.pi / 2)])
        cr = pos + self.radius * np.array([np.cos(theta - np.pi / 2), np.sin(theta - np.pi / 2)])

        return (cl, 1), (cr, -1)

    def get_path(self, circle, point):
        c = circle[0]

        # vectors from centers of the circles to position of the states
        cp = self.pos0 - c

        # vectors from centers of the circles to their respective tangents
        t = self.tangent(circle, point, self.radius)
        ct = t - c
        L = np.linalg.norm(point - t) # distance from t to point

        # compute arc(lengths)
        alpha_p = np.arctan2(cp[1], cp[0])
        alpha_t = np.arctan2(ct[1], ct[0])
        d_alpha = alpha_t - alpha_p # angle from pos to tangent around the circle1 center
        bool = (np.sign(d_alpha) != circle[-1])  # check if direction of angle and circle are not identical, eg not both CCW
        d_alpha = circle[-1] * abs(2 * np.pi * bool - abs(d_alpha)) # make sure the angle is measured along the direction of the turn

        total = abs(d_alpha * self.radius) + L

        return [alpha_p, alpha_t, t], [d_alpha, L, total]

    def make_path(self):
        """obtain dubins path
        
        Return
        
        path(np.array(l, n, m)) : list of passible paths from start configuration to given points
                                n = number of points in a path 
                                m = 3 (x,y,theta)
                                l = number of paths 
                                
                                note : the paths are sorted from shortest to longest
                            
        lengths(list)         : lenghts of the paths in ascending order 
        
        """
        
        self.paths = np.zeros((2, self.res, 5))
        self.paths[:][:][-2] += self.v
        lengths = np.array([])

        i = 0
        for circle in self.circles:
            path, length = self.get_path(circle, self.pos1)
            lengths = np.append(lengths, length[-1])

            if type(path[2]) == np.ndarray:
                alpha_samples = abs(self.res * length[0] * self.radius // length[-1]).astype(np.int64)
                L_samples = (self.res - alpha_samples).astype(np.int64)

                n = 0
                for a in np.linspace(path[0], path[0] + length[0], alpha_samples):
                    p = circle[0] + self.radius * np.array([np.cos(a), np.sin(a)])
                    th = a + circle[-1] * np.pi/2
                    self.paths[i][n][:2] += p
                    self.paths[i][n][2] += th
                    self.paths[i][n][-1] += circle[-1]*self.phi
                    n += 1

                V = self.pos1 - path[2]
                th = np.arctan2(V[1], V[0])
                for l in np.linspace(path[2], self.pos1, L_samples):
                    self.paths[i][n][:2] = l
                    self.paths[i][n][2] += th
                    n += 1

            i += 1

        # sort the paths from shortest to longest
        sorted = np.array([])
        sorted = np.append(sorted, [self.paths[np.argmin(lengths)], self.paths[np.argmax(lengths)]])

        self.paths = sorted.reshape((2, self.res, 5))
        self.actions = self.paths[:,:,3:]
        self.paths = self.paths[:,:,:3]
        return self.paths, np.sort(lengths), self.actions

    def plot(self):

        for [path, color] in zip(self.paths, ['lightblue', 'navajowhite']):
            circles = self.get_circles(path[-1][:2], path[-1][-1])
            self.plot_state(path[-1], circles, self.radius, color=color)
            plt.plot(path.T[0], path.T[1], '.')

        self.plot_state(self.start_conf, self.circles, self.radius, 'g')

        plt.plot(self.pos1[0], self.pos1[1], 'x', color='r')

        plt.gca().set_aspect(1)
        plt.show()


    def plot_circle(self,pos, R=1., color='r'):
        angles = np.arange(0, 2 * np.pi + 0.1, 0.1)
        plt.plot(R * np.cos(angles) + pos[0], R * np.sin(angles) + pos[1], '-.', color=color)

    def plot_state(self,q, circles, R=1., color='r'):
        # Plot the circles
        self.plot_circle(circles[0][0], R, color)
        self.plot_circle(circles[1][0], R, color)

        plt.plot(q[0], q[1], 'o', color=color) # plot robot center
        plt.quiver(q[0], q[1], np.cos(q[2]), np.sin(q[2]), scale=5, color=color) # plot arrow in the direction of orientation
        plt.plot(circles[0][0][0], circles[0][0][1], '+', color=color)
        plt.plot(circles[1][0][0], circles[1][0][1], '_', color=color)

    def rot2d(self,vec, angle):
        mat = np.array([[np.cos(angle), -np.sin(angle)],
                        [np.sin(angle), np.cos(angle)]])
        new = mat @ vec
        return new

    def tangent(self,circle, point, R=1.):
        V = point - circle[0]
        D = np.linalg.norm(V)

        if D > R:
            alpha = np.arccos(R/D)
            alpha *= -circle[-1]

            n = self.rot2d(V, alpha)  # vector pointing from circle center to tangent point
            n /= np.linalg.norm(n)

            tangent = circle[0] + R * n

            return tangent

        else:
            return np.inf



#################
# TESTS
#################

if __name__== '__main__' :
    np.random.seed()

    q0 = [np.random.randint(-5,5), np.random.randint(-5,5), np.random.random_sample()*2*np.pi]
    point = [np.random.randint(-5,5), np.random.randint(-5,5)]

    # q0 = [0, 0, 0]
    # point = [0,-2]

    radius = 1.5

    d = Dubin(q0, point, Rturn=radius, n_samples=25)

    paths, lengths, actions = d.make_path()

    print(paths, actions)
    d.plot()