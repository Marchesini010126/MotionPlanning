from packages.RobotPlanningRoutines.Dubin import Dubin
from packages.RobotPlanningRoutines.planners_and_env import Robot
import numpy as np
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import matplotlib.pyplot as plt

Rturn = 10
start      = (0, 0,90)  # start configuration

myrobot              = Robot(start[0],start[1],start[2],dimension=(45,25))

# DEFINE CAR MODEL
myrobot.set_car_spec(vel_max=40,max_yaw_rate=60*np.pi/180)                  # m/s
myrobot.set_baseline(baseline=45) # same as cal width

theta       = np.linspace(0,2*np.pi,100)
R           = np.linspace(0.1,100,300)

x           = np.outer(R,np.cos(theta)) 
y           = np.outer(R,np.sin(theta)) 
row,col     = np.shape(x)

grid        = np.empty((row,col))

for jj in range(row) :
    for ii in range(col):
    
       mydubin = Dubin(start, (x[jj,ii],y[jj,ii]), robot_obj=myrobot, n_samples=10)
       paths, lengths, actions = mydubin.make_path()
  
       grid[jj,ii]             = lengths[0] #save best path

fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
# Plot the surface.
surf = ax.plot_surface(x, y, grid, cmap=cm.coolwarm,
                       linewidth=0, antialiased=False)

# Customize the z axis.
ax.zaxis.set_major_locator(LinearLocator(10))
# A StrMethodFormatter is used automatically
ax.zaxis.set_major_formatter('{x:.02f}')

# Add a color bar which maps values to colors.
fig.colorbar(surf, shrink=0.5, aspect=5)

plt.show()

plt.contour(x,y,grid)
plt.show()
       
       



#mydubin = Dubin(start, goal, robot_obj=myrobot, n_samples=10)