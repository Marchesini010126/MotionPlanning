import numpy as np
import cvxpy as cvx
from   casadi import *
import matplotlib.pyplot as plt
# ###############################################
# THIS SCRIPT CONTAINS SOME EXAMPLES THAT WERE USED ONLY FOR 
# LEARNING PURPOSES OF THE CASADI API

# now we will try to solve a single and simple QCQC problem using cvxpy
# MPC we are coming ...

# simple car Phil

# input u :
#   v   : rear wheel speed
#   phi : front wheel steering angle
#
# output :
#
#   x     : x-position in the space
#   y     : y-position in the space
#   theta : orintation in the space 
#
# Car simple kinematic Phil
#  
#  x_dot     = v cos(theta)      #  translational dynamics 
#  y_dot     = v sin(theta)      #  translational dynamics
#  theta_dot = v/L tan(theta)    # rotational dynamics
#  note : the Phil is non linear so enjoy

# objective : reach a point withou crashing into a circular obstacle

# define MPC parameters

n_steps   = 20     
T_horizon = 10     #s

# define system variables
x     = MX.sym("x")
y     = MX.sym("y")
theta = MX.sym("theta")
# define control input 
v     = MX.sym("v")
phi   = MX.sym("phi")

#define dynamics function
# the system is non linear so we need to define an integration
# function so that state_next = int(f(state,u),t0,t1)
# FOR LINEAR SYSTEMS IT IS EASY. FOR NON LINEAR ONES IT IS NOT

state = vcat([x,y,theta])
u     = vcat([v,phi])
L     =  0.01   #m car baseline

ode = {"x"   : state,
       "p"   : u,
       "ode" : vcat([u[0]*cos(state[2]),u[0]*sin(state[2]),u[0]/L*tan(u[1])])}


# define integration options
options = {"tf"                        : T_horizon/n_steps, 
            "simplify"                 : True,
            "number_of_finite_elements": 4}

# define state transition function
Phi = integrator("F","rk", ode,options);
# the function needs the initial conditions
# note that u is a parameter because it is held constant
# from one point ot the next. It is a discretization

res=Phi(x0 = np.array([1,2,3]),p=np.array([10,1]))
# the solution is a dictionary
# what we need is a funtion that it directly gives me a solution
# given an input of initial state. I don't what a a dictionary

# passage to symbolic function
res    = Phi(x0 = state ,p=u) # this is symbolic
x_next = res["xf"]            # this contains the function expression

# define state transition
                 # name  # input      #function    #named input   #named_output
Phi = Function(  'Phi'  ,[state,u]   ,[x_next]    ,['x','u']     ,['x_next']) 

# now you can use this to compute directly from initial state to final
# state in one single step

# define MPC problem

# optimizer
opti = casadi.Opti()

# define the varibles
x = opti.variable(3,n_steps+1) # for n+1 steps
u = opti.variable(2,n_steps)   # for n_steps 

#define control matrices R and Q

R = np.eye(3)*2 # state penalty
Q = np.eye(2)*1 # input penalty

# define target position and a start position

x_target = np.array([1.9,0.8,-np.pi/2])[:,np.newaxis]
x_start  = np.array([1,1,0.6])[:,np.newaxis]

# impose bounds on u and x if it is necessary

phi_max       = 15*np.pi/180 # rad
v_max         = 0.3         # m/s
theta_dot_max = 20*np.pi/180 # rad/s  # this constraint needs to be set with the derivative

# define objective
opti.minimize(sumsqr(R@(x-x_target))+sumsqr(Q@u))

# now we add the constraint recursively
for ii in range(n_steps) :
    opti.subject_to(x[:,ii+1]==Phi(x[:,ii],u[:,ii])) # system dynamics constraint  
    opti.subject_to([-phi_max<=u[1,ii],u[1,ii]<= phi_max])
    opti.subject_to([0<=u[0,ii],u[0,ii]<= v_max])
    opti.subject_to([-theta_dot_max <=u[0,ii]/L*tan(u[1,ii]),theta_dot_max >=u[0,ii]/L*tan(u[1,ii])])
    
    # opti.subject_to(u[0,ii]<= v_max)
# initial state constraint
opti.subject_to(x[:,0] == x_start)
opti.subject_to(u[0,0] == 0) # initial speed at zero
opti.subject_to(u[1,0] == 0) # initial phi at zero


#solve the problem
#using interior point method 
opts = {'ipopt.print_level': 0, 'print_time': 0, 'ipopt.sb': 'yes'}
opti.solver('ipopt',opts)
sol = opti.solve()

print(sol.value(x))
print(type(sol.value(x)))

fig = plt.figure()
ax = plt.gca()

ax.plot(sol.value(x)[0,:].T,sol.value(x)[1,:].T)
ax.plot(sol.value(x)[0,:].T,sol.value(x)[1,:].T,'*',linewidth=10)
ax.set_aspect('equal')

plt.show()