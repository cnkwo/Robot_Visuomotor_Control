# Robot Vsiuomotor Control

By Patrick J Nkwocha

The aim of this project is to compose a program to catch a flying object (ball) thrown into a robot workspace with a UR10 arm, without any provided cartesian values as instructions. This project has been done inside the ROS ecosystem.

This is achieved by calling a function within my wrriten program (solution.py) - ***"findTrajectory()"*** within a class called - ***TrajectoryFinder***. This function performs a search algorithm which performs a search sequence to catch the ball

## How to run the code:

1. Inside the robot_control_simulation package open the file how to use the package and follow the instructions

2. Open a 3rd terminal (control + shift + T) and run the following:

    - rosrun robot_control_simulation ball_traj

In response, as input press 2 

3. Now everything is setup, inside the scripts folder open a 4th terminal and run the following:

    - rosrun robot_control_simulation solution
