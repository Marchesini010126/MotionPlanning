import numpy as np
import cvxpy as cvx

from   casadi import *
import matplotlib.pyplot as plt
import pygame



# define MPC problem


class Controller_mpc :
    
    def __init__(self) :
        pass
    
    def set_time_horizon(self,Ts=10):
        # horizon of the MPC controller
        self.T = Ts
    def set_n_steps(self,nsteps=20):
        #define number of the steps in the time horison
        self.n_steps = nsteps
    def set_car_baseline(self,L=0.30):
        #define car baseline
        self.L   = L
    def set_max_speed(self,vmax=1):
        self.vmax=vmax
    def set_min_speed(self,vmin=0):
            self.vmin=vmin
    def set_max_steering(self,phi=1):
        # in radians
            self.phi_max=phi
    def set_max_yaw_rate(self,maxyaw):
        # in radians/s
            self.maxyaw=maxyaw  
    def set_control_penalty(self,Q=np.eye(2)):
        # define control penalty matrix
        # for the mpc definition
        #  sum(x.T R x + u.T Q u)
        
        self.Q = np.sqrt(Q)
        # note -> the root square is taken because of 
        # the optimization problem formulation
    
    def set_state_penalty(self,R=np.eye(2)):
        # define state penalty matrix
        # for the mpc definition
        # sum(x.T R x + u.T Q u)
        
        self.R = np.sqrt(R)
        # note -> the root square is taken because of 
        # the optimization problem formulation
        
    def set_start_configuration(self,initial_state):
        # initial_state :  np.array([x,y,theta])   initial state of the system
        #                  
        #                 x     --> x-position [m]
        #                 y     --> y-position [m]
        #                 theta --> heading [rad]
            self.state=initial_state 
    
    def set_obstacles(self,obstacles_list=None) :
        # define obstacles dictionary with bounding radius
        # input :
        #
        # obstacles_list : list of dictionaries containing all the obstacles definitions
        #                  see obstacles factory for a full description 
        #                  of the obstacle dictionary definition
        
        if len(obstacles_list) == 0 :
           self.obstacles = [obstacles_list]
           
        else :
            self.obstacles = obstacles_list
        
    def init_controller(self):
        
        # define system variables
        x     = MX.sym("x")
        y     = MX.sym("y")
        theta = MX.sym("theta")
        
        #define control input 
        v     = MX.sym("v")
        phi   = MX.sym("phi")
        
        # define dynamics function
        # the system is non linear so we need to define an integration
        # function so that state_next = int(f(state,u),t0,t1)
        # FOR LINEAR SYSTEMS IT IS EASY. FOR NON LINEAR ONES IT IS NOT

        state = vcat([x,y,theta])
        u     = vcat([v,phi])
        
        ode = {"x"   : state,
                "p"   : u,
                "ode" : vcat([u[0]*cos(state[2]),u[0]*sin(state[2]),u[0]/self.L*tan(u[1])])}

        # define integration options
        options = {"tf"                        : self.T/self.n_steps, 
                    "simplify"                 : True,
                    "number_of_finite_elements": 4}

        # define state transition function
        Phi    = integrator("F","rk", ode,options) #define integration step
        res    = Phi(x0 = state ,p=u)              #define symbolic output
        x_next = res["xf"]                         # define symbolic output expression
        
        #define single input/output function
        Phi = Function(  'Phi'  ,[state,u]   ,[x_next]    ,['x','u']     ,['x_next']) 
        
        self.step_function = Phi
        
        # set optimiser for mcp control loop
        # optimizer
         # define optimization cost 
        
    def step(self,control) :
        # updates the state of the system given the control input 
        # should be given
        
        # Input 
        #
        # control       :  np.array([v,phi])
        #
        #                 v       --> speed          [m/s]
        #                 phi     --> steering angle [rad/s]
        #                 
        
        
        next_state = self.step_function(self.state,control)
        self.state = np.array(next_state)
    
    
    def mpc_optimal_control(self,target_state) :
        # take an mpc step toward the target using the curret state
        ## Input 
        #
        # target_state :  np.array([x,y,theta])   initial state of the system
        #                  
        #                 x     --> x-position [m]
        #                 y     --> y-position [m]
        #                 theta --> heading [rad]
        
        self.optimser = casadi.Opti()

        # define the varibles
        x = self.optimser.variable(3,self.n_steps+1) # for n+1 steps
        u = self.optimser.variable(2,self.n_steps)   # for n_steps 
        
        obstacles_cost = 0.0
        # add kinematic constraints
        for ii in range(self.n_steps) :
            self.optimser.subject_to(x[:,ii+1]==self.step_function(x[:,ii],u[:,ii])) # system dynamics constraint  
            self.optimser.subject_to([-self.phi_max<=u[1,ii],u[1,ii]<= self.phi_max])
            self.optimser.subject_to([self.vmin<=u[0,ii],u[0,ii]<= self.vmax])
            self.optimser.subject_to([-self.maxyaw<=u[0,ii]/self.L*tan(u[1,ii]),self.maxyaw >=u[0,ii]/self.L*tan(u[1,ii])])
            try :
                for obstacle in self.obstacles :
                    center = obstacle['center'][:,np.newaxis]
                    radius = obstacle['radius']
                    
                    #self.optimser.subject_to(sqrt((x[0,ii]-center[0])**2+(x[1,ii]-center[1])**2) > radius) # system dynamics constraint  
                    
            except  AttributeError :
                pass
        
        try :
           for obstacle in self.obstacles :
                    center = obstacle['center'][:,np.newaxis]
                    radius = obstacle['radius']
                    obstacles_cost += 1/1E-6/sumsqr(x[:2,:]-center)
        except  AttributeError :
                pass
            
        self.optimser.minimize(sumsqr(self.R@(x-target_state))+sumsqr(self.Q@u)+10*obstacles_cost)
        
            # self.optimser.subject_to(u[0,ii]<= v_max)
        # initial state constraint
        self.optimser.subject_to(x[:,0] == self.state)

        #solve the problem
        #using interior point method 
        opts = {'ipopt.print_level': 0, 'print_time': 0, 'ipopt.sb': 'yes'}
        s_opts = {'max_iter': 100}
        
        self.optimser.solver('ipopt',opts,s_opts)
        try :
           sol = self.optimser.solve()
        except :
           self.optimser.debug.show_infeasibilities()
           return
       
        control              = sol.value(u)[:,0]  # take only the first optimal control 
        self.predicted_state = sol.value(x)       # save predicted state
        return control
    
    def draw_predicted_state(self,screen):
        
        # plots the predicted trajectory 
        # over the prediction horizon
        
        for state in self.predicted_state.T :
            state = np.array(state)
           
            pygame.draw.circle(screen,(250,0,0),(state[0],state[1]),1)
           
        