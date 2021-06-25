## DD2410 Project information ##

Requires ROS installed (in Linux). 
The goal of this project was to implement a mission planner for [TIAGo](https://pal-robotics.com/robots/tiago/) to execute a set of tasks.
The TIAGo Robot was expected to pick an Aruco cube startring from an unknown positon and then move it to another table. The TIAGo must make use of its sensors and a prior map of the room to transport the cube safely among rooms. Furthermore, the robot was expected to be able to be "kidnapped" manually by an user and be able to start over in the task in such a way to eventually suceed with the mission. 


Simulation was done in [Gazebo](http://gazebosim.org/) of a of a [TIAGo](https://pal-robotics.com/robots/tiago/) robot in an apartment.

A behavior tree was implemented that goes through the following main states:

### Requirements ###
1. Robot has localized itself in the apartment
2. Navigation to picking pose
3. Cube detected
4. Complete picking task 
5. Navigation with cube to second table
6. Complete placing task
7. Cube placed on table?
  1. Yes: end of mission
  2. No: go back to state 2 and repeat until success. For this, you need to respawn the cube to its original pose in case it has fallen.


## Install ##

