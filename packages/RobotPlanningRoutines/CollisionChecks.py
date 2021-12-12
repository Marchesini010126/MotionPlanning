import pygame
import sys
import numpy as np
from   scipy.spatial import distance
from   random import randint


## simple circular collison check 
def CircleCollision(r1,r2,center1,center2) :
        # ----------DESCRIPTION--------------
        # check collison between two
        # circular objects in the space
        # -----------INPUT-------------------
        #  center1    center obs1      ---> np.array,list,tuple
        #  center2    center obs2      ---> np.array,list,tuple
        #  r1         radius 1         ---> float
        #  r2         radius 2         ---> float
        # -----------OUTPUT------------------
        #  True/False collision?
        # 
        check_collision = False
        c1    = np.array([center1[0],center1[1]])
        c2    = np.array([center2[0],center2[1]])
        Delta = np.linalg.norm(c1-c2)
        limit = r1+r2
        if Delta <= limit:
            check_collision=True
            
        return check_collision


# create support vector        
def support_vector(direction,points) :
    # this functions finds the support
    # vector in a given direction for a given
    # set of points. A support vector is the 
    # vector with highest component in a given direction
    
    # what is the function doing :
    # 1) define a direction (a vector not necessarely normalised)
    # 2) check what is vector that has the highest component 
    #    in that direction
    #    how ? simply use the dot product
    
    # input form
    # point     ---> list of 2d points. it can be list of tuple also
    #                (for pygame simplicity)
    #                the list is in the form (n,2) for 2d case
    #                n vectros in 2D space
    #
    # direction --->  needs to be a direction 2D
    #                 (list of tuple or list)
    # OUTPUT 
    #
    # support vector     np.array()
    
    
    if type(direction).__module__ != np.__name__ :
        direction = np.asarray(direction,dtype=np.float64)
    if type(points).__module__    != np.__name__ :
        points = np.asarray(points,dtype=np.float64)
    
    # accomodate for single point check
    if len(np.shape(points)) == 1: 
        support_vec = points
    
    else :
        projections   = np.sum(direction*points,axis=1) # list of projections
        support_indx  = np.argmax(projections) # always return only one index even if you have two equal                                   # max points
        support_vec   = points[support_indx,:]
    
    return support_vec


def minkowskiDifference(verticesA,verticesB):
    
    # ----------DESCRIPTION--------------
    # obtain minkowsky difference 
    # for two polygons
    # the difference is taken only at the 
    # vertices
    # -----------INPUT-------------------
    #  verticesA   first polygon vertices  -->  np.array((N,2))
    #  verticesB   first polygon vertices  -->  np.array((M,2))
    # -----------OUTPUT------------------
    # minkVert     minkowsky difference    -->  np.array((N*M,2))
    
    if len(np.shape(verticesA)) == 1 :
        minkVert = np.array([verticesA-vb for vb in verticesB])
    elif len(np.shape(verticesB)) == 1 :
        minkVert = np.array([va - verticesB for va in verticesA])
    else :   
        minkVert = np.array([va-vb for va in verticesA for vb in verticesB])
    
    return minkVert
    
    
def cross2Dvs2D(vector1,vector2):
    
    
    # ----------DESCRIPTION--------------
    # computes the cross product 
    # of 2D vectors
    # omega = vector1 X vector2
    # input
    # -----------INPUT-------------------
    # vector1  2D vector          ---> np.array(2,)
    # vector2  2D vector          ---> np.array(2,)
    # -----------OUTPUT------------------
    # omega out of plane component --> float
    
    
    omega = vector1[0]*vector2[1] - vector1[1]*vector2[0]
    return omega


def cross2Dvs3D(Omega,vector1):
    
    
    # ----------DESCRIPTION--------------
    # computes the cross product 
    # of 2D vectors and a 3D one
    # omega = Omgea X vector2
    # 
    # -----------INPUT-------------------
    # vector1  2D vector              ---> np.array(2,)
    # Omega    out of plane component ---> float   
    # (component of the 3D vector in the out of plane direction
    # -----------OUTPUT------------------
    # Omega X vectro1 2D vector       ---> np.array(2,)
    #
    # note : change the sign of one of you want to have
    #        the other order of the product
    
    vector = np.array([-vector1[1]*Omega,-(-vector1[0]*Omega)])
    return vector



def checkSimplex(simplex):
    
    # ----------DESCRIPTION--------------
    # computes the closest simplex to a give
    # origin location starting from an initial
    # simplex. IN addition it gives the 
    # search direction toward the origin
    # 
    # -----------INPUT-------------------
    # simplex   list of vertices   ---> [np.array(2,)]
    # -----------OUTPUT------------------
    # searchDirection                ---> np.array(2,)
    # inside      is the origin in   ---> 0/1 False/True
    #             the simplex
    # simplex    updated simplex     ---> [np.array(2,)]
    
    
    
    # CASE 1 : simplex has dimension 2
    if len(simplex) == 2:
        
        A  = simplex[-1]  # fist is the last added
        B  = simplex[0]   # 
        AB = B-A          # vector from A to B
        AO = -A           # vector from A to origin
        
        searchDir = cross2Dvs3D(cross2Dvs2D(AB,AO),AB) # new search direction
        
        if np.sum(AB*AO)>0 :
            simplex = [B,A]
            inside  = 0 
            return simplex,searchDir,inside
        
        else :
            
            simplex = [A]
            inside  = 0
            return simplex,AO,inside
        
    if len(simplex) == 3:
        
        A  = simplex[2]  # fist in
        B  = simplex[0]   # last out
        C  = simplex[1]   # vector from A to B
        AC = C-A
        AB = B-A
        AO = -A           # vector from A to origin
        
        searchDir1 = cross2Dvs3D(cross2Dvs2D(AC,AB),AB)  # new search direction 
        searchDir2 = cross2Dvs3D(cross2Dvs2D(AC,AB),-AC) # new search direction

        if np.sum(AO*searchDir1)>0 :
            if np.sum(AO*AB)>0 :
                simplex = [A,B] 
                inside  = 0
                return simplex,searchDir1,inside 
            
            else :
                simplex  = A
                inside   = 0
                return simplex,AO,inside 
            
        if np.sum(AO*searchDir2)>0 :
            if np.sum(AO*AC)>0 :
                simplex = [A,C] 
                inside  = 0
                return simplex,searchDir2,inside 
            
            else :
                simplex  = A
                inside   = 0
                return simplex,AO,inside 
        
        # if it happens that all the previous
        # check fail, than the center is inside 
        # the simplex
        else :
            inside = 1
            return [],[],inside
        
        
        
             
        
def GJK(verticesA,verticesB,radius=0):
    # ----------DESCRIPTION--------------
    # GJK collision check algorithm
    # 
    # -----------INPUT-------------------
    #  verticesA      first polygon vertices  -->  np.array((N,2))
    #  verticesB      first polygon vertices  -->  np.array((M,2))
    #  radius         radius of the circle    -->  float (non-negative)
    #                 in case polygon vs circle         
    #
    #  note --> radius : in case a circular obstacle is checked agains a polygon
    # -----------OUTPUT------------------
    # searchDirection                ---> np.array(2,)
    # inside      is the origin in   ---> 0/1 False/True
    #             the simplex
    # simplex    updated simplex     ---> [np.array(2,)]
    
    # note 
    minkdiff          = minkowskiDifference(verticesA,verticesB)
    initial_direction = minkdiff[0,:]
    
    A                 = support_vector(initial_direction,verticesA) - support_vector(-initial_direction,verticesB)
    simplex           = [A] # initialise simplex list of vectors
    D                 = -A
    inside   = 0
    max_iter = 1000
    counter  = 0
    while counter < max_iter or inside ==1:
        
        A       = support_vector(D,minkdiff)
        if np.sum(A*D)<0  :
            versor      = D/np.linalg.norm(D)
            minDistance = abs(np.sum(A*versor))
            
            if minDistance >= radius:
            # no intersection in this case:
            # you already moved in the straight direction
            # to the origin and you didn't pass over it 
            # so you won't pass it later
            # note : at final iteration abs(A*D) is the 
            # distance between the two closest points
            
              return 0 # collision. Stop and exit
        
            else : 
              return 1 # circle vs polygon collison
        
        
        # order of the simplex ---> LIFO
        # the last element is the new one
        simplex.append(A)
        simplex,D,inside=checkSimplex(simplex)
        
        if inside :
            return 1
        
        counter = counter +1
