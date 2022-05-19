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

**module3** ``ObstaclesFactory.py``


| functions       | Description                                |
|-----------------|--------------------------------------------|
| createNpolygon  | creates a N faces polygon                  |
| createRectangle | creates a rectangle obstacle               |
| createCircle    | create a circular obstacle                 |
| rotationMatrix  | returns a rotation matrix for 2D rotations |
| rotation        | rotatetes a polygon of a given angle       |


# Video result

https://user-images.githubusercontent.com/71294988/149579085-4589d134-c944-4b85-a55e-130f899ebf50.mp4

# Aknowledgments
Thanks to Algorobotics for inspiring the RRT algorithm structure 
https://www.youtube.com/watch?v=Tllz7Ox2B3g



