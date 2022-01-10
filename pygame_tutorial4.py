import pygame
import sys
from   random import randint
import numpy as np
from   scipy.spatial import distance

pygame.init()
pygame.font.init() # init font module
                   
collision_check        = pygame.font.SysFont('Times', 30) 
collision_mess_surface = collision_check.render('Collision', False, (0, 0, 255))

class Robot():
    
    def __init__(self,x0=0,y0=0,thrust=10,type='circle',dimension=20) :
        self.x0      = x0
        self.y0      = y0
        self.vx0     = 0.0
        self.vy0     = 0.0
        self.thrust  = thrust
        self.state   = np.array([self.x0,self.y0])
        self.dt      = 0.01 #s
        self.mass    = 10
        self.gravity = 9.81 #m/s2
        self.type    = 'type'   
        available_shapes = {'rectangle','circle'}  
        
        if type in available_shapes :
            if type == 'rectangle':
                self.width   = dimension[0]
                self.height  = dimension[1]
            else:
                if isinstance(dimension,list) or isinstance(dimension,tuple):
                    raise ValueError('For circular robot you can''t define a list of dimension.\n Just use an float')
                self.radius = dimension
                
        
    def motion(self) :
        # equations of motion for the mall
        vmax = 20  # m/s # upper limit of the speed
        vmin = 0   # m/s # loer limit of the speed
        self.vx0 = self.vx0 + self.thrust/self.mass*self.dt
        self.vy0 = self.vy0 + self.gravity*self.dt
        # check speed
        if np.abs(self.vx0) > vmax :
            self.thrust  = 0
        if np.abs(self.vy0) > vmax :
            self.gravity  = 0 
            
        
        self.x0  = self.x0  + self.vx0*self.dt
        self.y0  = self.y0  + self.vy0*self.dt
        
        
    def check_border(self,width,height) :
        if self.x0< self.radius  or self.x0>width-self.radius:
            self.thrust = -self.thrust
            self.vx0     = -self.vx0
        if self.y0< self.radius  or self.y0 >height-self.radius:
            self.gravity = -self.gravity
            self.vy0     = -self.vy0
        
        
    def draw_robot(self,screen):
        color  = (10,230,100)
        center = (self.x0,self.y0)
        pygame.draw.circle(screen,color,center,self.radius)
    
    def draw_laser(self,screen,centers):
        # centers          --> list of touple 
        color=(255,0,0)
        for center in centers :
            robot_pos  = (self.x0,self.y0)
            pygame.draw.line(screen,color,(self.x0,self.y0),center)
            
            
    def check_collision(self,screen,centers,obstancles_radius)  :
        # centers          --> list of touple 
        # obtscles radious --> list of integers
        obs_pos    = np.array([[center[0],center[1]] for center in centers ])
        robot_pos  = np.array([[self.x0,self.y0]])
        distances  = distance.cdist(robot_pos,obs_pos).reshape(len(centers),1)
        
        if self.type == 'recatangle':
             metric = np.sqrt((self.width/2)**2+(self.height/2)**2)
        else:
             metric = self.radius 

        for dist in distances :
            # to be changed to be able to work with lists of radius
            if dist < np.array([obstancles_radius+metric]) :
                screen.blit(collision_mess_surface,(0,0))
    
    def check_collision_AABB(self,) :
        # Axis Aligned Bounding Boxes
        # to start with let's try only for rectangluar obstacles against rectangluar obstacles
        
        
        pass


class CreateRandomObtacles():
    def __init__(self,n_obstacles,obstacle_type,Xrange,Yrange) :
        
        available_obtscales = {'rectangle','circle'}
        if not obstacle_type  in available_obtscales:
            raise ValueError('{} not in the obstacles list:\nTry rectangle or circle'.format(obstacle_type))
        
        self.obs_type    = obstacle_type
        self.n_obstacles = n_obstacles
        self.Xrange      = Xrange    # list of touple (min x max x)
        self.Yrange      = Yrange    # list of touple (min y max y)
        
        # to be changed later on, but for now fix obstacles size
        
        self.radius      = 30 # radius for circular obstacles
        self.width       = 60 # width  for rectangular obstacles
        self.height      = 60 # height for rectangular obstacles 
    
    
    def create_obstacles(self) :
        
        # list of touple for the centers
        self.centers     = [(randint(self.Xrange[0],self.Xrange[1]),randint(self.Yrange[0],self.Yrange[1])) for jj in range(self.n_obstacles)]
        #  list of dimensions 
        if self.obs_type == 'rectangle' :
            dimesions = [(randint(0,self.width),randint(0,self.height)) for jj in range(self.n_obstacles)]
        else :
            dimesions = [randint(0,self.radius) for jj in range(self.n_obstacles)]
        colors  = [(randint(0,255),randint(0,255),randint(0,255)) for jj in range(n_obstacles)]
        
        # create dictionary 
        # key    --> obstacle number
        # item   --> [(center),(color),(dimensions)]
        #              touple ,touple , touple or scalar
        obstacles_dict = {jj:[self.centers[jj],colors[jj],dimesions[jj]] for jj in range(n_obstacles)}
        
        return obstacles_dict
        
    def draw_obstacles(self,obstacles_dict,screen) :
        
        if self.obs_type=='rectangle' :
            for n_obs,obstacle in obstacles_dict.items():
               pygame.draw.rect(screen,obstacle[1],pygame.Rect(obstacle[0],obstacle[2]))
               
        else :
            for obstacle in obstacles_dict:
                   pygame.draw.circle(screen,obstacle[1],obstacle[0],obstacle[2])
            
        
               
        
# create basic environment 
screen_width  = 800 
screen_height = 600
n_obstacles   = 10
obs_radius    = 50 # you can even make a list if you 
                   # want to have a specific radious for each
screen        = pygame.display.set_mode((screen_width, screen_height)) 
clock         = pygame.time.Clock() # clock object to monitor data framing 
running       = True # infinite loop initialization

# create random rectangular obstacles
obstacles_set      = CreateRandomObtacles(n_obstacles,'rectangle',[0,screen_width],[0,screen_height])
obstacles_set_dict = obstacles_set.create_obstacles() # obtain obstacles list

# define circual obstacles centers
centers = [(randint(0,screen_width),randint(0,screen_height)) for jj in range(n_obstacles)]
# define circular obstacles colors on the screen 
colors  = [(randint(0,255),randint(0,255),randint(0,255)) for jj in range(n_obstacles)]

centers_rect = [(randint(0,screen_width),randint(0,screen_height)) for jj in range(n_obstacles)]

myrobot1 = Robot(600,100) # initial position of the robot 1
myrobot2 = Robot(300,50)  # initial position of the robot 2

while running:
    # check if quit was pressed
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
           running = False
           pygame.quit()
           sys.exit()
    
    # reset the image  
    screen.fill((0, 0, 0))
    # update the screen
    for center,color in zip(centers,colors):
        pygame.draw.circle(screen,color,center,obs_radius)
        
    myrobot1.check_border(screen_width,screen_height)
    myrobot1.motion()
    myrobot1.draw_robot(screen)
    myrobot1.draw_laser(screen,centers)
    myrobot1.check_collision(screen,centers,obs_radius)
    
    myrobot2.check_border(screen_width,screen_height)
    myrobot2.motion()
    myrobot2.draw_robot(screen)
    myrobot2.draw_laser(screen,centers)
    myrobot2.check_collision(screen,centers,obs_radius)
    
    obstacles_set.draw_obstacles(obstacles_set_dict,screen)
    
    pygame.display.flip()    
    screen.fill((0,0,0))       