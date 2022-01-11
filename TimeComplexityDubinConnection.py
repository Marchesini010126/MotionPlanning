
from packages.RobotPlanningRoutines.Dubin import Dubin
from packages.RobotPlanningRoutines.planners_and_env import Robot,readPathDataFromTxt
import numpy as np
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import matplotlib.pyplot as plt
import time


Rturn      = 10
start      = (0, 0,np.pi)  # start configuration
myrobot              = Robot(start[0],start[1],start[2],dimension=(45,25))
point      = (30,30)




clock_Dubin=[]
clock_eucl = []
mydubin = Dubin(start, (30,30), Rturn=1, n_samples=10)
for N in range(10,2500,20) :
    print(N)
    t_start = time.time()
    for jj in range(N) :
        mydubin.make_path()
    t_end = time.time()
    clock_Dubin.append(t_end-t_start)
    
    t_start = time.time()
    
    for jj in range(N) :
        np.sqrt((point[0]-start[0])**2 + (point[1]-start[1])**2)
    t_end = time.time()
    clock_eucl.append(t_end-t_start)



clock_Dubin= np.array(clock_Dubin)
clock_eucl = np.array(clock_eucl)

fig,ax = plt.subplots()

ax.plot(np.arange(10,2500,20),clock_Dubin,c='r',label='Dubin Dist')
ax.plot(np.arange(10,2500,20),clock_eucl,c='b',label='Euclide Dist')
ax.set_ylabel('time [s]')
ax.set_xlabel('N')
ax.set_yscale('log')
ax.legend()
plt.show()
