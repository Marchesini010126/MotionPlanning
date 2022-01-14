## check functionality of dubins path function
import numpy as np
from packages.RobotPlanningRoutines.Dubin import Dubin

np.random.seed()

q0 = [np.random.randint(-5,5), np.random.randint(-5,5), np.random.random_sample()*2*np.pi]
point = [np.random.randint(-5,5), np.random.randint(-5,5)]

# q0 = [0, 0, 0]
# point = [0,-2]

radius = 1.5

d= Dubin(q0, point, Rturn=radius, n_samples=25)

paths,legths,actions = d.make_path()

print(paths,legths)
print(actions)
d.plot()