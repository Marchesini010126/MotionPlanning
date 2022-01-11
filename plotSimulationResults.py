import matplotlib.pyplot as plt
from packages.RobotPlanningRoutines.planners_and_env import readPathDataFromTxt
import sys,os
import numpy as np

sim_dir = "./OutputSimulationsGJKcollision"

active_GJK_lenght = []
notactive_GJK_length = []
time_active = []
time_notactive = []

for jj,file in enumerate(os.listdir(sim_dir)) :
    
    flag = file.split('/')[-1].split('_')[-1].split('.')[0]
    
    solution = readPathDataFromTxt(os.path.join(sim_dir,file))
    print(solution)
    if not solution[0]['length'] ==  None :
        if flag == 'active' :
             active_GJK_lenght.append(solution[-1]['length'])
             time_active.append(solution[-1]['time'])
        elif flag == 'NotActive' :
             notactive_GJK_length.append(solution[-1]['length'])
             time_notactive.append(solution[-1]['time'])

active_GJK_lenght = np.array(active_GJK_lenght)
notactive_GJK_length = np.array(notactive_GJK_length)
time_active = np.array(time_active)
time_notactive = np.array(time_notactive)


fig,ax = plt.subplots(2,1)
ax[0].plot(np.arange(len(active_GJK_lenght)),active_GJK_lenght,marker='o',c='r',label='GJK')
ax[0].plot(np.arange(len(notactive_GJK_length)),notactive_GJK_length,marker='o',c='b',label= 'CircleCollision')
ax[1].plot(np.arange(len(active_GJK_lenght)),time_active,marker='o',c='r',label='GJK')
ax[1].plot(np.arange(len(notactive_GJK_length)),time_notactive,marker='o',c='b',label= 'CircleCollision')


ax[0].set_xlabel('test number')
ax[1].set_xlabel('test number')
ax[0].set_ylabel('path length [cm]')
ax[1].set_ylabel('time required [s]')
ax[0].legend(loc='upper left')
ax[1].legend(loc='upper left')
plt.show()
fig.savefig('GJK_vs_Circle.png')

