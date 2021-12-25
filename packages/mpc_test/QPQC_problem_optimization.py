import numpy as np
import cvxpy as cvx
from casadi import *
import matplotlib.pyplot as plt

# now we will try to solve a single and simple QCQC problem using cvxpy
# MPC we are coming ...

# simple car model

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
# Car simple kinematic model
#  
#  x_dot     = v cos(theta)      #  translational dynamics 
#  y_dot     = v sin(theta)      #  translational dynamics
#  theta_dot = v/L tan(theta)    # rotational dynamics
#  note : the model is non linear so enjoy

# objective : reach a point withou crashing into a circular obstacle

# define initial conditions
x0      = 1
y0      = 1
theta0  = 180
state_0 = np.array([x0,y0,theta0])[:,np.newaxis]
target  = np.array([10,10,0])[:,np.newaxis] # target configuration

# steps in the optimization
n_steps   = 30 
T_horizon = 15 #s

# define symbolic variables to be optimsed

state = MX.sym("state",3,1)  # define symbolic state 
u     = MX.sym("u",2,1)      # define symbolic control
print(state,u)

# casadi definition of a function

expression = state[1] * u[0] # this is not a function but an expression

myfun = Function("myfun",[state,u],[expression])
aa=myfun(np.array([2.,3.,5.]).T,np.array([2.,3.]).T)
print(aa)

# now it is time to implement the dynamic model of our system
# the model we use is non-linear.
# for this reason it is essential tu built a dynamic integrator
# that is able to tell the system how to take the next step for the system
# For linear problems it is quite easy. But for non-linear problems it is not

# x_dot = f(x,u)
# x_k+1 = F(x,u) --> this is what we need to obtain at the end 
# note that during the integartion time, the input is considered constant 
# this is essential for understanding the basic principle



# now we create an ode solver using a dictionary to specify
# all the inputs to the solver. The keys are established

# define model
L = 3
ode_expression = [u[0]*cos(state[2]),u[0]*sin(state[2]),u[0]/L*tan(u[1])]
odefun         = Function("ode",[state,u],ode_expression)


x = MX.sym("x")
y = MX.sym("y")
theta = MX.sym("theta")

v = MX.sym("v")
phi = MX.sym("phi")

ode_initialization = {"x"   : vcat([x,y,theta]),
                      "p"   : vcat([v,phi]),
                      "ode" : vcat([v*cos(theta),v*sin(theta),v/L*tan(theta)])}


# define integration options
options = {"tf"                        : T_horizon/n_steps, 
            "simplify"                 : True,
            "number_of_finite_elements": 4}



F = integrator("F","rk", ode_initialization,options);
print('look here at the function form', F)
# no in order to evaluate the function it is conveniente to 
# use the default form as you can check from the print

res=F(x0 = np.array([1,2,3]),p=np.array([10,1]))
# the solution is a dictionaty
print('result:', res["xf"])

# the function can also be called symbolically using starting conditons that are 
# symbolix

state = vcat([x,y,theta])
u     = vcat([v,phi])  
res=F(x0 = state ,p=u)
print("symbolic result",res['xf'])
# Now the value of the key is a call to 
# the integrator class. I can than simplify the 
# normal structure of the fucntion so
# tha from an input I get an output directly 
# and I eliminate any intemediate structure

x_next = res["xf"]
     
                 # name  # input   #function  #named input   #named_output
model = Function('model' ,[state,u]   ,[x_next]    ,['x','u']     ,['x_next']) 

# NOTE : the input must coincide with the symbolic variable used in the evaluation
#        of the symbolic res

# now you can use this to compute directly from initial state to final
# state 

direct_result=model(np.array([1,1,1])[:,np.newaxis],np.array([1,2])[:,np.newaxis])
print('Direct result is ',direct_result)
print('this is the mode ',model)

# Now if it is desired to give multiple steps for the input 
# it is possible to do it using the mapaccup function

# now we need to solve an mpc problem
# the way to do that is by using the optimization class
# this will help us solve the problem

opti = casadi.Opti()

# define the varibale
# this are matrices to be used

x = opti.variable(3,n_steps+1)
print(x.shape)
u = opti.variable(2,n_steps)

# objective function

#define control matrices R and Q

R = np.eye(3)
Q = np.eye(2)

opti.minimize(sumsqr(x)+sumsqr(u))
x_start = np.array([3,4,1.3])
# now we add the constraint recursively
for ii in range(n_steps) :
    # note that at index 30 you are already at element 31
    opti.subject_to(x[:,ii+1]==model(x[:,ii],u[:,ii])) 
    # add the no slip condition
    opti.subject_to(u[:,ii])
    
#single time constraints 
opti.subject_to(x[:,0] == x_start )

print(opti)
opti.solver('ipopt')
sol = opti.solve()

print(sol.value(x))
print(type(sol.value(x)))
plt.plot(sol.value(x)[0,:].T,sol.value(x)[1,:].T)
plt.show()