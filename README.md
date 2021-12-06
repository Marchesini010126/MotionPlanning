# MotionPlanning
Motion Planning and Decision Making course project

## Main Objective
Implement a map environment with the module``pygame`` in order to test the main **Robot Motion Planning** algorithms
## Structure
The project is subdivided in three main modules 

 **module1** ``planners_and_env.py``
 This module constains the main classes used to create the robot that will move in the map and initialise its state. 

| Class  | Description                                              |
|--------|----------------------------------------------------------|
| EnvMap | Used to draw the obstacles in the ``pygame`` environment |
| Robot  | Initialise robot shape and Equations of Motion           |


 **module2** ``CollisionCheck.py``
 Implemenatation of simple collision detetection algorithms 
 
| functions       | Description                                                |
|-----------------|------------------------------------------------------------|
| CircleCollision | check collisions between two circular objects in the space |
| GJK             | implementation of the GJK algorithm                        |

For more info on GJK algorithm check the following links

[52 minutes GJK description] (https://caseymuratori.com/blog_0003)

[GJK wikipidia] (https://en.wikipedia.org/wiki/Gilbert%E2%80%93Johnson%E2%80%93Keerthi_distance_algorithm)

**module2** ``ObstaclesFactory.py``


| functions       | Description                                |
|-----------------|--------------------------------------------|
| createNpolygon  | creates a N faces polygon                  |
| createRectangle | creates a rectangle obstacle               |
| createCircle    | create a circular obstacle                 |
| rotationMatrix  | returns a rotation matrix for 2D rotations |
| rotation        | rotatetes a polygon of a given angle       |

In order to understand how this module works it is essential to understand how an obstacle is defined . In the current version of the project, an obstacle is a dictionary constaing the following key/item pairs


| Key        | item                                                                            |
|------------|---------------------------------------------------------------------------------|
| 'center'   | obstacle center coordinates                                                     |
| 'vertices' | list of vertices for the polygon  note : equal to radius for circular obstacles |
| 'color'    | RGB color                                                                       |
| 'radius'   | bounding radius of the polygon                                                  |
| 'type'     | 'polygon' or 'circle'                                                           |


## A closer look at the Robot class

The robot class initialise a simple robot that can be used to follow the path created by planning algoritm. The idea behind the class in the following :

The robot instance needs to be initialised with an initial state (speed and position). The state will be then updated during the simulation. The simulation time goes from t<sub>0</sub> to t<sub>f</sub> with discreet time step dt.  At every dt the Robot receives a input from defined ``controller_fuction`` which takes as an input the state of the robot and a reference path to follow and it outputs the control action **u**,
The control action is a  vector which will then given as an input to the robot calss method ``motion()`` . Inside ``motion`` the equations of motion of the robot are defined and the new state at time t+dt is updateted after the input control is given to the robot

 Robot Class

[![](https://mermaid.ink/img/eyJjb2RlIjoiZ3JhcGggVERcbiAgICBBW0NocmlzdG1hc10gLS0-fEdldCBtb25leXwgQihHbyBzaG9wcGluZylcbiAgICBCIC0tPiBDe0xldCBtZSB0aGlua31cbiAgICBDIC0tPnxPbmV8IERbTGFwdG9wXVxuICAgIEMgLS0-fFR3b3wgRVtpUGhvbmVdXG4gICAgQyAtLT58VGhyZWV8IEZbZmE6ZmEtY2FyIENhcl1cbiAgIiwibWVybWFpZCI6eyJ0aGVtZSI6ImRhcmsifSwidXBkYXRlRWRpdG9yIjp0cnVlLCJhdXRvU3luYyI6dHJ1ZSwidXBkYXRlRGlhZ3JhbSI6dHJ1ZX0)](https://mermaid-js.github.io/mermaid-live-editor/edit#eyJjb2RlIjoiZ3JhcGggVERcbiAgICBBW0NocmlzdG1hc10gLS0-fEdldCBtb25leXwgQihHbyBzaG9wcGluZylcbiAgICBCIC0tPiBDe0xldCBtZSB0aGlua31cbiAgICBDIC0tPnxPbmV8IERbTGFwdG9wXVxuICAgIEMgLS0-fFR3b3wgRVtpUGhvbmVdXG4gICAgQyAtLT58VGhyZWV8IEZbZmE6ZmEtY2FyIENhcl1cbiAgIiwibWVybWFpZCI6IntcbiAgXCJ0aGVtZVwiOiBcImRhcmtcIlxufSIsInVwZGF0ZUVkaXRvciI6dHJ1ZSwiYXV0b1N5bmMiOnRydWUsInVwZGF0ZURpYWdyYW0iOnRydWV9)

mermaid
graph LR
A[planner] -->B((+))-->C[controller] --u-->D[robot] --> E[current state] -->B


## To Do List
Here are some applications that still need to be  implemented
1. Expand the GJK algorithm so that it can work Circle vs Polygon
2. Implement the ``Robot.motion()`` method
3.  Create a function that creates random obstacles, starting from the obstacles primitives
4. implement RRT algorithm to be added to the ``planners_and_env.py`` module
5. implement Dubins path algorithm to be added to the ``planners_and_env.py`` module
6. **ESSENTIAL**  test all the functions like collison checks functions 
