
import matplotlib.pyplot as plt
from packages.RobotPlanningRoutines.planners_and_env import readPathDataFromTxt
import sys,os
import numpy as np

"""read output from a standard path txt file as computed by the RRT planner class"""
"""dictionary with solutions returned in descending order from less optimal to optimal"""

"""sol = [{'path_number': ..., 'length':....,'time':....,'planner':....}]"""

file_names  = ['./sims_norebase.txt','./sims_rebase.txt']
names       = ['RRT standard','RRT improved']
colors      = ['blue','red']

fig,ax = plt.subplots(2,1)

for kk,file_name,color,name in zip(range(2),file_names,colors,names) :
    
    f       = open(file_name,'r')
    lines   =  f.readlines()
    counter = 0     # number of paths
    counter_sim = 0  # simulation number

    save_output = [] # list of dictionaries
    book = []
    for line in lines :
        original_line = line
        
        if "RRT search Summary" == original_line.strip() : 
            save_output.insert(counter_sim,book)
            book         =  []
            counter_sim  +=  1 
            counter       =   0
            
        
        if line.strip() == 'NO PATH TO GOAL FOUND' :
            book = [{'path_number': None, 'length':None,'time':None,'planner':lines[7].split(':')[1].strip()}]
            
        line = line.split(':')
        
        if len(line)>1 :
            if "Path number"==line[0].strip() :
        
                book.insert(counter,{'path_number': float(line[1]),'planner':lines[7].split(':')[1].strip()})
            if "Path length"==line[0].strip() :
            
                book[counter]['length']=float(line[1])
            if "Time required"==line[0].strip() :
                book[counter]['time']=float(line[1].split('s')[0])
                counter += 1
            
            
    f.close()


    path_lengths = []
    time = []
    for jj,book in enumerate(save_output[1:]) :
        
        solution = book[0]
        
        if not solution['length'] ==  None :
            path_lengths.append(solution['length'])
            time.append(solution['time'])


    path_lengths = np.array(path_lengths)[:15]
    time         = np.array(time)[:15]

    
    ax[0].scatter(np.arange(len(path_lengths)),path_lengths,c=color,label=name)
    ax[0].plot(np.arange(len(path_lengths)),path_lengths,c=color)
    ax[1].scatter(np.arange(len(path_lengths)),time,c=color,label= name)
    ax[1].plot(np.arange(len(path_lengths)),time,c=color)


ax[0].set_xlabel('test number')
ax[1].set_xlabel('test number')
ax[0].set_ylabel('path length [cm]')
ax[1].set_ylabel('time required [s]')
ax[0].legend(loc='upper left')
ax[1].legend(loc='upper left')
plt.show()
fig.savefig('First_path_comparison_map7.png')

