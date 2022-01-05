## check functionality of dubins path function
import numpy as np
from RobotPlanningRoutines.Dubin import Dubin

np.random.seed()

q0 = [np.random.randint(-5,5), np.random.randint(-5,5), np.random.random_sample()*2*np.pi]
point = [np.random.randint(-5,5), np.random.randint(-5,5)]

# q0 = [0, 0, 0]
# point = [0,-2]

radius = 1.5

d = Dubin(q0, point, radius, 25)

paths,legths = d.make_path()

print(paths,legths)
d.plot()