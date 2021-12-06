import pygame
import sys
import numpy as np

from   scipy.spatial import distance
from   random import randint


# ObstaclesFactory
# info : Creates a sets sets of random obtscles

# obstacle :
# 
# It is a dictionary constaining the key words that
# are required to define the obstacle in space 
# and define its color 
#
# keywords ---> 'center'     center                              tuple
#               'vertices'   vertices                            np.array
#               'color'      color                               tuple (R,G,B)
#               'radius'     bounding radious for the obstacle   float
#               'type'       {circle,polygon}                    string
#                     
# note : for circular obstacles the vertices variable is equal to the center location
#        and the bounding radious is euqal to the radious itself


# Description
#
# INPUT
#
#
# OUTPUT

def createNpolygon(N,r,center=np.array([0,0]),color=(250,0,0)) :
    
    
    # ----------DESCRIPTION--------------
    # Creates regular polygon with N faces
    # that is inscribed in a circle of size r
    #
    # -----------INPUT-------------------
    # center  polygon center           ---> list,tuple or array (2,)
    #  N      number of vertices       ---> float
    #  color  color                    ---> touple  (3,)
    #  r      bounding radius          ---< float
    # -----------OUTPUT------------------
    # obsracle dictionary
    
    
    xc = center[0]
    yc = center[1]
    
    polygon=np.array([[xc+r*np.cos(2*np.pi/N*ii),yc+r*np.sin(2*np.pi/N*ii)] for ii in range(N)])
    obstacle = {'center': center,'vertices':polygon,'color':color,'radius':r,'type':'polygon'}
    return obstacle

def createRectangle(width,height,center=np.array([0,0]),color=(250,0,0)) :
    
    # ----------DESCRIPTION--------------
    # creates list of vertices of a rectangle
    # the output is a list of touple with all the 
    # the vertices
    #
    # -----------INPUT-------------------
    #  center   polygon center         ---> list,tuple or array (2,)
    #  width                           ---> float
    #  height                          ---> float
    #  color                           ---> touple  (3,)
    # -----------OUTPUT------------------
    # obsracle dictionary

    
    xc   = center[0]
    yc   = center[1]
    
    a1   = [xc + width/2,yc -height/2]
    a2   = [xc + width/2,yc +height/2]
    a3   = [xc - width/2,yc +height/2]
    a4   = [xc - width/2,yc -height/2]
    
    rad  = np.sqrt(width**2+height**2)
    rect = np.array([a1,a2,a3,a4])
    obstacle = {'center': np.array([xc,yc]),'vertices':rect,'color':color,'radius':rad,'type':'polygon'}
    return obstacle 


def createCircle(radius,center=np.array([0,0]),color=(250,0,0)) :
    # ----------DESCRIPTION--------------
    # creates circular obstacles
    #
    # -----------INPUT-------------------
    #  center   polygon center         ---> list,tuple or array (2,)
    #  radius                          ---> float
    #  color                           ---> touple  (3,)
    # -----------OUTPUT------------------
    # obsracle dictionary
    obstacle = {'center': np.array([center[0],center[1]]),'vertices':np.array([radius]),'color':color,'radius':radius,'type':'circle'}
    return obstacle
def rotationMatrix(angle) :
    # ----------DESCRIPTION--------------
    # creates 2D rotation matrix
    #
    # -----------INPUT-------------------
    #  angle   in radians               ---> float
    # -----------OUTPUT------------------
    # rotation matrix                   ---> np.array(2,2)
    # 
    rotmat = np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])
    return rotmat
    
def rotation(obstacle,angle) :
    
    # ----------DESCRIPTION--------------
    # rotates an obstacle of an angle 
    # specified by the user
    # -----------INPUT-------------------
    #  angle     in radians               ---> float
    #  obstacle  obstacle dictionary      ---> dict
    # -----------OUTPUT------------------
    #  obstacle  obstacle dictionary      ---> dict
    # 
    
    
    if obstacle['type'] == 'circle' :
        raise ValueError('You cannot rotate a circle')
    else :
        rotmat = rotationMatrix(angle)
        
        vertices = obstacle['vertices']
        center   = obstacle['center']
        
        centered_vertices    = vertices - center
        rotated_vertices     = (rotmat@centered_vertices.T).T + center
        obstacle['vertices'] = rotated_vertices
        return obstacle
        
    

def create_obstacles_set():
    # TODO : create set of obstacles
    # input --> list oc centers
    #       --> dimsions 
    pass
