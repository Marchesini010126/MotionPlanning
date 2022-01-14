# MotionPlanning
Motion Planning and Decision Making course project

## Main Objective
Implement from scratch an RRT algorithm for path planning under differential constarint using a Dubin car model.

## Structure
The project is subdivided in three main modules 

 **module1** ``planners_and_env.py``
 This module constains the main classes used to create the robot, the RRT planner and the map environment. 

| Class  | Description                                              |
|--------|----------------------------------------------------------|
| **EnvMap** | Used to draw the obstacles in the ``pygame`` environment |
| **Robot**  | Initialise robot shape and Equations of Motion           |
| **RRTplanner**  | Initialise robot shape and Equations of Motion           |


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


How is an obstacle defined?

An obstacle is a dictionary constaing the following key/item pairs


| Key        | item                                                                            |
|------------|---------------------------------------------------------------------------------|
| 'center'   | obstacle center coordinates                                                     |
| 'vertices' | list of vertices for the polygon  note : equal to radius for circular obstacles |
| 'color'    | RGB color        (tuple)                                                               |
| 'radius'   | bounding radius of the polygon                                                  |
| 'type'     | 'polygon' or 'circle'                                                           |


How is an robot defined?

The robot class initialise a simple robot that can be used to follow the path created by planning algoritm. The idea behind the class in the following.

This robot model is used as input to the RRTplanner so that dubins path could be computed accoding to the minimum trun radius of the robot and collision checking functions can take the dimensions of the car into account. The planner also computes the inputs required to follow the path and the actuation time. The robot model is a simple non-differential car kinematic model.


Check the simple script ``TestArena.py`` in order to get an idea of how can you load maps (both random maps and pre-computed structured maps).

# A nice video showing the result of our project

